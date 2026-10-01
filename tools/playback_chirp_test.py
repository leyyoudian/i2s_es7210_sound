#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Play coherent FM chirp bursts (bird-call-like, 2-6kHz) through PC speakers
while capturing ESP32 serial, for DOA direction-consistency testing."""
import math
import struct
import subprocess
import time
import wave

import serial

PORT = "COM26"
BAUD = 115200
SR = 48000
BURST_MS = 350
GAP_MS = 1650
N_BURSTS = 10
OUT_WAV = "tools/play_chirp.wav"
OUT_LOG = "tools/chirp_capture.log"


def make_chirp_wav():
    frames = []
    for b in range(N_BURSTS):
        frames.extend([0.0] * int(SR * GAP_MS / 1000))
        n = int(SR * BURST_MS / 1000)
        for i in range(n):
            env = min(1.0, i / (0.01 * SR), (n - i) / (0.01 * SR))
            t = i / SR
            # 2->6kHz 指数扫频, 重复段内两次扫频 (鸟鸣感, 相干相位结构)
            frac = (t / (BURST_MS / 1000.0 * 0.5)) % 1.0
            f = 2000.0 * (6000.0 / 2000.0) ** frac
            phase = 2 * math.pi * f * t
            frames.append(0.55 * env * math.sin(phase))
    with wave.open(OUT_WAV, "w") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SR)
        w.writeframes(b"".join(struct.pack("<h", int(s * 32767)) for s in frames))
    print(f"wrote {OUT_WAV}")


def main():
    make_chirp_wav()
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    log = open(OUT_LOG, "w", encoding="utf-8", errors="replace")
    t0 = time.time()
    proc = subprocess.Popen(
        ["powershell", "-NoProfile", "-Command",
         f"(New-Object Media.SoundPlayer '{OUT_WAV}').PlaySync()"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = t0 + (N_BURSTS * (BURST_MS + GAP_MS) + 4000) / 1000
    while time.time() < deadline:
        data = ser.read(4096)
        if data:
            log.write(f"[{time.time()-t0:7.2f}] " +
                      data.decode("utf-8", errors="replace"))
            log.flush()
    proc.wait()
    ser.close()
    log.close()
    spans = [((b * (BURST_MS + GAP_MS) + GAP_MS) / 1000.0,
              (b * (BURST_MS + GAP_MS) + GAP_MS + BURST_MS) / 1000.0)
             for b in range(N_BURSTS)]
    print("chirp spans (s):", [f"{a:.1f}-{b:.1f}" for a, b in spans])


if __name__ == "__main__":
    main()
