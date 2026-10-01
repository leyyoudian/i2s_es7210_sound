#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Capture ESP32 serial log to a file for offline analysis."""
import sys, time, argparse
import serial

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM23")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--secs", type=float, default=45.0)
    ap.add_argument("--out", default="tools/capture.log")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    t_end = time.time() + args.secs
    n = 0
    with open(args.out, "w", encoding="utf-8", errors="replace") as f:
        while time.time() < t_end:
            data = ser.read(4096)
            if not data:
                continue
            text = data.decode("utf-8", errors="replace")
            f.write(text)
            f.flush()
            n += len(text)
    ser.close()
    print(f"captured {n} chars -> {args.out}")

if __name__ == "__main__":
    main()
