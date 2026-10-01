#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Play a real music WAV through PC speakers while capturing ESP32 serial."""
import pathlib
import subprocess
import sys
import time

import serial

PORT = "COM26"
BAUD = 115200
WAV = str(pathlib.Path(__file__).resolve().parent / "play_music.wav")
OUT_LOG = "tools/music_capture.log"
PAD_SECS = 6.0


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    log = open(OUT_LOG, "w", encoding="utf-8", errors="replace")
    t0 = time.time()
    proc = subprocess.Popen(
        ["ffplay", "-nodisp", "-autoexit", "-loglevel", "error", WAV],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = t0 + dur + PAD_SECS
    while time.time() < deadline:
        data = ser.read(4096)
        if data:
            log.write(f"[{time.time()-t0:7.2f}] " +
                      data.decode("utf-8", errors="replace"))
            log.flush()
    proc.wait()
    ser.close()
    log.close()
    print(f"music spans ~2.0-{2.0+dur:.0f}s -> {OUT_LOG}")


if __name__ == "__main__":
    main()
