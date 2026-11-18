"""Capture simultaneous PCM snapshots from an explicit -D DOA_PCM_DUMP=ON build.

Each WAV is a separate contiguous 1024-frame snapshot, not a continuous stream.
Rebuild with -D DOA_PCM_DUMP=OFF and flash after diagnostics.
"""
import argparse
from pathlib import Path
import re
import time
import wave
import winsound

import serial


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--seconds', type=float, default=22)
    ap.add_argument('--speaker-wav', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    pcm = bytearray()
    index = 0
    reading = False
    with serial.Serial(args.port, 115200, timeout=.2) as ser:
        ser.reset_input_buffer()
        winsound.PlaySound(str(args.speaker_wav.resolve()),
                           winsound.SND_FILENAME | winsound.SND_ASYNC | winsound.SND_LOOP)
        start = time.monotonic()
        try:
            while time.monotonic() - start < args.seconds:
                line = ser.readline().decode('utf-8', 'replace').strip()
                if 'PCM: BEGIN' in line:
                    pcm = bytearray()
                    reading = True
                elif 'PCM: END' in line and reading:
                    if len(pcm) == 8192:
                        path = args.out / f'frame-{index:03d}.wav'
                        with wave.open(str(path), 'wb') as wav:
                            wav.setnchannels(4)
                            wav.setsampwidth(2)
                            wav.setframerate(48000)
                            wav.writeframes(pcm)
                        print(path, flush=True)
                        index += 1
                    reading = False
                elif reading and 'PCM: ' in line:
                    payload = line.split('PCM: ', 1)[1].strip()
                    if re.fullmatch(r'(?:[0-9a-fA-F]{2}\s+)*[0-9a-fA-F]{2}', payload):
                        pcm.extend(bytes.fromhex(payload))
        finally:
            winsound.PlaySound(None, 0)
    print(f'Captured {index} complete snapshots')


if __name__ == '__main__':
    main()
