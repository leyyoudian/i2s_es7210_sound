"""Play one continuous real WAV and measure the DOA stream over time.

No repeated snippets. Reports use aligned frame timestamps where available.
The dominant direction measures stability, not accuracy against a surveyed angle.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import re
import time
import wave
import winsound

import serial


def parse_row(line, received, offset):
    if 'DOA=' not in line:
        return None, offset
    values = dict(re.findall(r'([a-zA-Z_]+)=(-?\d+(?:\.\d+)?)', line))
    if 'frame_ms' in values:
        device = float(values['frame_ms']) / 1000
        candidate = received - device
        offset = candidate if offset is None else min(offset, candidate)
        frame = device + offset
    else:
        frame = received
    return dict(time=frame, received=received,
                angle=float(values.get('DOA', 0)), raw=float(values.get('raw', 0)),
                active=int(values.get('act', 0)), display=int(values.get('dsp', 0)),
                pixel=int(values.get('led', -1)), valid='status=valid' in line,
                confidence=float(values.get('conf', 0)),
                instant_analysis=int(values.get('fast', -1)),
                sustained_frames=int(values.get('steady', -1)),
                led_lit=int(values.get('lit', -1)),
                overflow=int(values.get('ovf', 0)), processing_us=int(values.get('us', 0))), offset


def summary(rows):
    if not rows:
        return dict(rows=0)
    valid = [r for r in rows if r['valid']]
    angles = [r['angle'] for r in valid]
    mean = None
    rms_error = None
    opposite = None
    if angles:
        mean = math.degrees(math.atan2(sum(math.sin(math.radians(a)) for a in angles),
                                       sum(math.cos(math.radians(a)) for a in angles))) % 360
        rms_error = math.sqrt(sum(((a-mean+180)%360-180)**2 for a in angles)/len(angles))
        opposite = sum(abs((a-mean+180)%360-180) > 120 for a in angles)
    pixels = [r['pixel'] for r in rows if r['display'] and r['pixel'] >= 0]
    # Event-triggered diagnostics are sampled more often than steady frames.
    # Integrate states over elapsed time so percentages are not biased by that.
    weights = [max(0., min(.5, b['time']-a['time'])) for a,b in zip(rows,rows[1:])]
    weights.append(weights[-1] if weights else .02)
    total = sum(weights) or 1.
    # Compare sector switching at a common four-Hz cadence across firmware.
    sampled=[]
    at=rows[0]['time']
    i=0
    while at<=rows[-1]['time']:
        while i+1<len(rows) and rows[i+1]['time']<=at: i+=1
        sampled.append(rows[i]['pixel'] if rows[i]['display'] else -1)
        at+=.25
    lit_samples=[p for p in sampled if p>=0]
    return dict(rows=len(rows), valid_rows=len(valid),
                active_fraction=sum(r['active'] for r in rows)/len(rows),
                display_fraction=sum(r['display'] for r in rows)/len(rows),
                mean_angle=mean, circular_rms_spread=rms_error, opposite_rows=opposite,
                displayed_pixels=dict(Counter(pixels)),
                observed_pixel_changes=sum(a!=b for a,b in zip(pixels,pixels[1:])),
                time_weighted_active_fraction=sum(w*r['active'] for w,r in zip(weights,rows))/total,
                time_weighted_display_fraction=sum(w*r['display'] for w,r in zip(weights,rows))/total,
                sampled_4hz_pixel_changes=sum(a!=b for a,b in zip(lit_samples,lit_samples[1:])),
                overflow_start=rows[0]['overflow'], overflow_end=rows[-1]['overflow'])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--wav', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    with wave.open(str(args.wav)) as wav:
        duration = wav.getnframes()/wav.getframerate()
    rows, pending, offset = [], b'', None
    with serial.Serial(args.port, 115200, timeout=.03) as ser, args.out.open('w',encoding='utf8') as log:
        ser.reset_input_buffer()
        drain_end, ready_end, ready = time.monotonic()+3, time.monotonic()+30, False
        while time.monotonic()<drain_end or (not ready and time.monotonic()<ready_end):
            pending += ser.read(ser.in_waiting or 1)
            while b'\n' in pending:
                raw,pending=pending.split(b'\n',1)
                row,offset=parse_row(raw.decode('utf8','replace'),time.monotonic(),offset)
                ready |= row is not None
        if not ready:
            raise RuntimeError('No DOA task diagnostics')
        pending=b''
        epoch=time.monotonic()
        next_report=30
        log.write(f'PLAYBACK duration={duration} host_epoch={epoch} wav={args.wav}\n')
        print(f'Playing {duration:.1f} seconds of continuous real music',flush=True)
        winsound.PlaySound(str(args.wav.resolve()),winsound.SND_FILENAME|winsound.SND_ASYNC)
        try:
            while time.monotonic()-epoch<duration+2:
                pending += ser.read(ser.in_waiting or 1)
                while b'\n' in pending:
                    raw,pending=pending.split(b'\n',1)
                    line=raw.decode('utf8','replace').strip()
                    received=time.monotonic()
                    row,offset=parse_row(line,received,offset)
                    frame_time=row['time']-epoch if row else received-epoch
                    if line:
                        log.write(f'[{frame_time:.3f}] recv={received-epoch:.3f} {line}\n')
                    if row and 0<=frame_time<=duration:
                        row['time']=frame_time
                        row['received']=received-epoch
                        rows.append(row)
                if time.monotonic()-epoch>=next_report and next_report<=duration:
                    recent=[r for r in rows if next_report-30<=r['time']<next_report]
                    print(json.dumps(dict(seconds=next_report,**summary(recent))),flush=True)
                    log.flush()
                    next_report+=30
        finally:
            winsound.PlaySound(None,0)
    report=dict(duration=duration,summary=summary(rows),rows=rows)
    args.out.with_suffix('.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print('TOTAL',json.dumps(report['summary']),flush=True)


if __name__=='__main__':
    main()
