"""Download public real recordings, play them, and capture DOA diagnostics.

ESC-50 recordings: https://github.com/karolpiczak/ESC-50
Spoken digits: https://github.com/Jakobovski/free-spoken-digit-dataset
Cached media and attribution live in .fetch/real-audio; no generated sounds.
Use the IDF Python (3.11, pyserial). --prepare downloads only.
"""
import argparse
import audioop
import base64
import concurrent.futures
import csv
import hashlib
import io
import json
from pathlib import Path
import re
import time
import urllib.request
import wave
import winsound

import serial

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / '.fetch/real-audio'


def github_file(repo, path):
    url = f'https://api.github.com/repos/{repo}/contents/{path}'
    request = urllib.request.Request(url, headers={'User-Agent': 'Local-DOA-audio-bench'})
    with urllib.request.urlopen(request, timeout=30) as response:
        item = json.load(response)
    return base64.b64decode(item['content'])


def prepare():
    CACHE.mkdir(parents=True, exist_ok=True)
    csv_path = CACHE / 'esc50.csv'
    if not csv_path.exists():
        csv_path.write_bytes(github_file('karolpiczak/ESC-50', 'meta/esc50.csv'))
    rows = list(csv.DictReader(io.StringIO(csv_path.read_text(encoding='utf-8'))))
    selections = []
    for category in ('dog', 'chirping_birds', 'clapping', 'door_wood_knock', 'coughing', 'clock_tick'):
        for row in [r for r in rows if r['category'] == category][:2]:
            selections.append(dict(name=f'{category}-{row["filename"]}',
                                   repo='karolpiczak/ESC-50', path='audio/' + row['filename'],
                                   attribution=f'https://freesound.org/s/{row["src_file"]}/',
                                   license='ESC-50: see dataset README and original Freesound attribution'))
    for digit in (0, 3, 8):
        selections.append(dict(name=f'speech-digit-{digit}', repo='Jakobovski/free-spoken-digit-dataset',
                               path=f'recordings/{digit}_jackson_0.wav',
                               license='CC BY-SA 4.0 (Free Spoken Digit Dataset)'))

    def fetch(item):
        path = CACHE / (item['name'] + '.wav')
        if not path.exists():
            path.write_bytes(github_file(item['repo'], item['path']))
        item['file'] = str(path)
        item['source'] = f'https://github.com/{item["repo"]}/blob/master/{item["path"]}'
        item['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        print('Ready:', item['name'], flush=True)
        return item

    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        items = list(pool.map(fetch, selections))
    (CACHE / 'manifest.json').write_text(json.dumps(items, ensure_ascii=False, indent=2), encoding='utf-8')
    return items


def playback_file(item, gain, channel='both'):
    original = Path(item['file'])
    with wave.open(str(original)) as wav:
        data = wav.readframes(wav.getnframes())
        channels, width, rate = wav.getnchannels(), wav.getsampwidth(), wav.getframerate()
    if width != 2:
        data = audioop.lin2lin(data, width, 2)
    if channels == 2:
        data = audioop.tomono(data, 2, .5, .5)
    elif channels != 1:
        raise ValueError('Expected mono/stereo recording')
    data, _ = audioop.ratecv(data, 2, 1, rate, 48000, None)
    # Normalize peak only; preserve the recorded waveform and its pauses.
    peak = audioop.max(data, 2)
    if peak:
        data = audioop.mul(data, 2, .30 * gain * 32767 / peak)
    duration = len(data) / 96000
    if channel != 'both':
        data = audioop.tostereo(data, 2, 1 if channel == 'left' else 0,
                               1 if channel == 'right' else 0)
    path = original.with_name(original.stem + f'-play-{gain:.2f}-{channel}.wav')
    with wave.open(str(path), 'wb') as wav:
        wav.setparams((1 if channel == 'both' else 2, 2, 48000, 0, 'NONE', 'not compressed'))
        wav.writeframes(data)
    return path, duration


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--prepare', action='store_true')
    ap.add_argument('--manifest', type=Path, help='Use an explicit cached real-recording suite')
    ap.add_argument('--port')
    ap.add_argument('--offset', type=int, default=0)
    ap.add_argument('--count', type=int, default=6)
    ap.add_argument('--gain', type=float, default=1.)
    ap.add_argument('--channel', choices=['both','left','right'], default='both')
    ap.add_argument('--out', type=Path, default=ROOT / 'tools/real_capture.log')
    ap.add_argument('--compact', action='store_true', help='Print counts instead of full per-frame arrays')
    ap.add_argument('--repeat', type=int, default=1, help='Repeat the selected real recordings with separate clip markers')
    args = ap.parse_args()
    manifest = args.manifest or CACHE / 'manifest.json'
    items = json.loads(manifest.read_text(encoding='utf-8')) if manifest.exists() else prepare()
    if args.prepare:
        return
    if not args.port:
        ap.error('--port is required for playback')
    report = []
    selected=[]
    for run in range(max(1,args.repeat)):
        for item in items[args.offset:args.offset+args.count]:
            selected.append(dict(item,name=item['name']+(f'#r{run+1}' if args.repeat>1 else '')))
    with serial.Serial(args.port, 115200, timeout=.02) as ser, args.out.open('w', encoding='utf-8') as log:
        ser.reset_input_buffer()
        # reset_input_buffer only clears the host. Drain the device-side USB
        # FIFO and the firmware logger queue before aligning a new playback.
        drain_end = time.monotonic() + 3
        ready_deadline = time.monotonic() + 30
        ready = False
        clock_offset = None
        sync_pending = b''
        while time.monotonic() < drain_end or (not ready and time.monotonic() < ready_deadline):
            sync_pending += ser.read(ser.in_waiting or 1)
            while b'\n' in sync_pending:
                raw, sync_pending = sync_pending.split(b'\n', 1)
                if b'DOA=' in raw:
                    ready = True
                match = re.search(rb'frame_ms=(\d+)', raw)
                if match:
                    value = time.monotonic() - int(match[1]) / 1000
                    clock_offset = value if clock_offset is None else min(clock_offset, value)
        if not ready:
            raise RuntimeError('No DOA diagnostics received; firmware/audio task is not ready')
        pending = b''
        epoch = time.monotonic()
        log.write(f'SYNC clock_offset={clock_offset} host_epoch={epoch:.6f}\n')
        try:
            for item in selected:
                path, duration = playback_file(item, args.gain, args.channel)
                log.write(f'CLIP {json.dumps(dict(name=item["name"],start=time.monotonic()-epoch,duration=duration,gain=args.gain,channel=args.channel))}\n')
                winsound.PlaySound(str(path), winsound.SND_FILENAME | winsound.SND_ASYNC)
                playback_start = time.monotonic()
                deadline = playback_start + duration + 1.5
                lines = []
                lit_during = 0
                while time.monotonic() < deadline:
                    pending += ser.read(ser.in_waiting or 1)
                    while b'\n' in pending:
                        raw, pending = pending.split(b'\n', 1)
                        line = raw.decode('utf-8', 'replace').strip()
                        if line:
                            captured = time.monotonic()
                            match = re.search(r'frame_ms=(\d+)', line)
                            if match:
                                device_time = int(match[1]) / 1000
                                value = captured - device_time
                                clock_offset = value if clock_offset is None else min(clock_offset, value)
                                frame_time = device_time + clock_offset
                            else:
                                frame_time = captured
                            log.write(f'[{frame_time-epoch:.3f}] recv={captured-epoch:.3f} {line}\n')
                            in_clip = playback_start - .03 <= frame_time <= playback_start + duration + .3
                            if in_clip:
                                lines.append(line)
                            if in_clip and 'dsp=1' in line:
                                lit_during += 1
                log.flush()
                valid = [line for line in lines if 'status=valid' in line]
                angles = [float(re.search(r'DOA=([\d.]+)', line)[1]) for line in valid]
                pixels = [int(re.search(r'led=(-?\d+)', line)[1]) for line in valid if 'led=' in line]
                result = dict(name=item['name'], gain=args.gain, channel=args.channel, duration=duration,
                              active=sum('act=1' in line for line in lines), valid=len(valid),
                              display_rows=lit_during,
                              angles=angles, pixels=pixels, source=item['source'])
                report.append(result)
                shown=result
                if args.compact:
                    shown={k:v for k,v in result.items() if k not in ('angles','pixels','source')}
                    shown['angle_range']=[min(angles),max(angles)] if angles else None
                print(json.dumps(shown, ensure_ascii=False), flush=True)
        finally:
            winsound.PlaySound(None, 0)
    args.out.with_suffix('.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
