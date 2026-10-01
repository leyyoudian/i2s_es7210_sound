#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Play band-limited noise bursts through PC speakers while capturing ESP32 serial.

Produces tools/play_test.wav + tools/play_capture.log; bursts are marked by
timestamps so the log can be split into burst / silence frames afterwards.
"""
import math
import random
import struct
import subprocess
import sys
import time
import wave

import serial

PORT = "COM23"
BAUD = 115200
SR = 48000
BURST_MS = 400
GAP_MS = 1600
N_BURSTS = 8
OUT_WAV = "tools/play_test.wav"
OUT_LOG = "tools/play_capture.log"


def make_wav():
    rnd = random.Random(42)
    samples = []
    for b in range(N_BURSTS):
        samples.extend([0.0] * int(SR * GAP_MS / 1000))
        # 2-6kHz 白噪声突发 (模拟响指/鸟叫频段), 带上升/下降沿避免爆音
        n = int(SR * BURST_MS / 1000)
        for i in range(n):
            env = min(1.0, i / (0.02 * SR), (n - i) / (0.02 * SR))
            t = i / SR
            # 简易带通: 白噪 + 差分 + 单极低通合成
            v = rnd.uniform(-1, 1)
            samples.append(0.5 * env * v)
    # 后处理: 简单高通+低通把能量集中到 2-6kHz
    out = []
    hp_prev = lp_prev = prev_x = 0.0
    a_hp = 0.85      # ~1.3kHz HP @48k
    a_lp = 0.45      # ~5.5kHz LP @48k
    for x in samples:
        hp = a_hp * (hp_prev + x - prev_x)
        lp = lp_prev + a_lp * (hp - lp_prev)
        hp_prev, prev_x, lp_prev = hp, x, lp
        out.append(max(-1.0, min(1.0, lp * 6.0)))
    with wave.open(OUT_WAV, "w") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SR)
        w.writeframes(b"".join(struct.pack("<h", int(s * 32767)) for s in out))
    print(f"wrote {OUT_WAV}")


def main():
    make_wav()
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
                      data.decode("utf-8", errors="replace").replace("\n", "\n"))
            log.flush()
    proc.wait()
    ser.close()
    log.close()
    burst_spans = [((b * (BURST_MS + GAP_MS) + GAP_MS) / 1000.0,
                    (b * (BURST_MS + GAP_MS) + GAP_MS + BURST_MS) / 1000.0)
                   for b in range(N_BURSTS)]
    print("burst spans (s):", [f"{a:.1f}-{b:.1f}" for a, b in burst_spans])


if __name__ == "__main__":
    main()
