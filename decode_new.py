#!/usr/bin/env python3
"""
Fixed live decoder:

1. Wait for ASCII TIME line:
       TIME yyyy-mm-dd HH:MM:SS
2. After that, treat all incoming bytes as binary.
3. Decode compact format safely.
"""

import serial
import time
import re
import binascii
from datetime import datetime, timedelta

PORT = "COM6"
BAUD = 115200
OUT_CSV = "decoded.csv"

BASE_MS = 5000
FAST_MS = 200

OPC_TS  = 0xF0
OPC_E   = 0xE5
OPC_END = 0xEE


# ----------------- DECODER --------------------

def bucket_to_temp_mid_c(bucket):
    if bucket <= 0: return 22.0
    if bucket >= 12: return 24.0
    return 22.5 + 0.5*(bucket-1)

def decode_compact(data):
    rows = []
    segments = [i for i,b in enumerate(data) if b == OPC_TS]

    seg_no = 0
    for ts_off in segments:
        seg_no += 1
        off = ts_off + 1

        if off + 4 > len(data):
            continue

        t_ms = (data[off]
                | (data[off+1]<<8)
                | (data[off+2]<<16)
                | (data[off+3]<<24))
        off += 4

        time_ms = t_ms
        in_exc = False
        idx = 0

        while off < len(data):
            b = data[off]
            off += 1

            if b == OPC_END:
                break
            if b == OPC_E:
                in_exc = not in_exc
                continue
            if b == OPC_TS:
                if off+4 > len(data): break
                time_ms = (data[off]
                           | (data[off+1]<<8)
                           | (data[off+2]<<16)
                           | (data[off+3]<<24))
                off+=4
                continue

            hi = (b>>4) & 0x0F
            lo = b & 0x0F

            tm = bucket_to_temp_mid_c(hi)
            rows.append((seg_no, idx, time_ms, hi, tm, int(in_exc)))
            idx+=1
            time_ms += FAST_MS if in_exc else BASE_MS

            tm = bucket_to_temp_mid_c(lo)
            rows.append((seg_no, idx, time_ms, lo, tm, int(in_exc)))
            idx+=1
            time_ms += FAST_MS if in_exc else BASE_MS

    return rows


# ----------------- MAIN --------------------

def main():
    print("Waiting for TIME line...")

    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(1)

    start_dt = None
    raw = bytearray()

    # Step 1: wait for TIME line (ASCII)
    while start_dt is None:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        print("[SERIAL]", line)

        if line.startswith("TIME "):
            # TIME 2025-11-23 17:55:43
            m = re.match(r"TIME (\d+-\d+-\d+) (\d+:\d+:\d+)", line)
            if m:
                start_dt = datetime.strptime(m.group(1)+" "+m.group(2),
                                             "%Y-%m-%d %H:%M:%S")
                print("Parsed TIME:", start_dt)
                print("Now collecting binary data...")
                break

    # Step 2: read BINARY ONLY
    last = time.time()
    while True:
        b = ser.read(1024)
        if b:
            raw.extend(b)
            last = time.time()
            print(f"Received {len(b)} bytes → total {len(raw)}")
        else:
            if time.time() - last > 0.5 and len(raw) > 0:
                break

    print("Binary transfer ended.")
    print("First 32 bytes (hex):", binascii.hexlify(raw[:32]))

    rows = decode_compact(bytes(raw))
    print("Decoded", len(rows), "samples.")

    # Step 3: write CSV
    with open(OUT_CSV, "w") as f:
        f.write("segment,index,time_ms,real_datetime,bucket,temp_mid_C,in_excursion\n")
        for seg,i,t_ms,bucket,temp,exc in rows:
            dt = start_dt + timedelta(milliseconds=t_ms)
            f.write(f"{seg},{i},{t_ms},{dt},{bucket},{temp:.2f},{exc}\n")

    print("CSV saved:", OUT_CSV)


if __name__ == "__main__":
    main()
