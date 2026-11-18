"""Repeatable PC-speaker/serial DOA bench. Uses moderate WAV amplitude.

Run with the IDF Python: doa_bench.py --port COM25 --out tools/before.log
Keep speaker and array stationary between before/after runs. Physical angle
and distance must be recorded separately; this bench measures repeatability.
"""
import argparse
import json
import math
from pathlib import Path
import random
import re
import struct
import time
import wave
import winsound

import serial


def make_wav(path):
    fs = 48000
    signal = []
    spans = []
    rng = random.Random(918)

    def silence(seconds):
        signal.extend([0.0] * round(seconds * fs))

    def burst(name, seconds, kind):
        start = len(signal) / fs
        hp = lp = prev = phase = 0.0
        count = round(seconds * fs)
        for i in range(count):
            env = min(1.0, i / (0.002 * fs), (count - i) / (0.002 * fs))
            if kind == 'noise':
                x = rng.uniform(-1, 1)
                hp = 0.90 * (hp + x - prev)
                lp += 0.48 * (hp - lp)
                prev = x
                value = lp * 0.65
            else:
                frequency = 1800 + 3600 * ((i / fs / 0.12) % 1.0)
                phase += 2 * math.pi * frequency / fs
                value = 0.18 * math.sin(phase)
            signal.append(value * env)
        spans.append(dict(name=name, start=start, end=len(signal) / fs))

    silence(2)
    burst('sustained_noise', 6, 'noise')
    silence(2)
    for i in range(4):
        burst(f'chirp_{i}', .35, 'chirp')
        silence(1.15)
    for i in range(8):
        burst(f'click_{i}', .020, 'noise')
        silence(.98)
    silence(2)
    with wave.open(str(path), 'wb') as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(fs)
        wav.writeframes(b''.join(struct.pack('<h', round(x * 32767)) for x in signal))
    return spans, len(signal) / fs


def summarize(rows, spans):
    report = []
    for span in spans:
        selected = [(t, line) for t, line in rows
                    if span['start'] <= t <= span['end'] + .4]
        valid = []
        raw = []
        pixels = []
        for _, line in selected:
            a = re.search(r'DOA=([\d.]+).*raw=([\d.]+)', line)
            if a:
                raw.append(float(a[2]))
                if 'status=valid' in line:
                    valid.append(float(a[1]))
            p = re.search(r'led=(-?\d+)', line)
            if p:
                pixels.append(int(p[1]))
        angle = None
        spread = None
        if valid:
            angle = math.degrees(math.atan2(sum(math.sin(math.radians(a)) for a in valid),
                                           sum(math.cos(math.radians(a)) for a in valid))) % 360
            spread = max(abs((a - angle + 180) % 360 - 180) for a in valid)
        report.append(dict(**span, rows=len(selected), valid=len(valid),
                           angle=angle, max_deviation=spread, raw=raw, pixels=pixels))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    wav = args.out.with_suffix('.wav').resolve()
    spans, duration = make_wav(wav)
    rows = []
    pending = b''
    with serial.Serial(args.port, 115200, timeout=.025) as ser, args.out.open('w', encoding='utf-8') as log:
        ser.reset_input_buffer()
        drain_end = time.monotonic() + 3
        while time.monotonic() < drain_end:
            ser.read(ser.in_waiting or 1)
        start = time.monotonic()
        winsound.PlaySound(str(wav), winsound.SND_FILENAME | winsound.SND_ASYNC)
        try:
            while time.monotonic() - start < duration + .5:
                pending += ser.read(ser.in_waiting or 1)
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    t = time.monotonic() - start
                    line = line.decode('utf-8', 'replace').strip()
                    if line:
                        rows.append((t, line))
                        log.write(f'[{t:.3f}] {line}\n')
        finally:
            winsound.PlaySound(None, 0)
    report = summarize(rows, spans)
    args.out.with_suffix('.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    for item in report:
        print(item['name'], 'valid=', item['valid'], 'angle=', item['angle'], 'spread=', item['max_deviation'])


if __name__ == '__main__':
    main()
