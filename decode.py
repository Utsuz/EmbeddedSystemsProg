#!/usr/bin/env python3
import argparse, sys
from typing import List, Tuple

OPC_TS  = 0xF0
OPC_E   = 0xE5
OPC_END = 0xEE

def parse_args():
    ap = argparse.ArgumentParser(description="Decode compact bitstream (first timestamp only, 4-bit buckets).")
    ap.add_argument("infile", help="Input file (binary). Use - for stdin.")
    ap.add_argument("--base", type=int, default=500, help="BASE_PERIOD_MS (default: 500)")
    ap.add_argument("--fast", type=int, default=25,  help="FAST_PERIOD_MS (default: 25)")
    ap.add_argument("--csv",  default="-", help="Output CSV file (default: stdout)")
    return ap.parse_args()

def read_all(path: str) -> bytes:
    if path == "-":
        return sys.stdin.buffer.read()
    with open(path, "rb") as f:
        return f.read()

def u32le(b: bytes, off: int) -> Tuple[int, int]:
    if off + 4 > len(b): raise ValueError("Unexpected EOF reading u32")
    v = b[off] | (b[off+1] << 8) | (b[off+2] << 16) | (b[off+3] << 24)
    return v, off + 4

def find_header(data: bytes) -> int:
    """Find the start index of the compact stream header: C1 01 ?? ?? 0D."""
    for i in range(0, max(0, len(data) - 4)):
        if data[i] == 0xC1 and data[i+1] == 0x01:
            # We don't strictly require bucket_count==13 here; decoder warns later.
            return i
    return -1

def bucket_to_temp_mid_c(bucket: int) -> float:
    """
    Map bucket -> midpoint °C using 0.5°C steps centered at:
      22.5, 23.0, 23.5, 24.0, 24.5, ..., 28.0
    Buckets:
      0  -> represent as 22.0 (below range clamp)
      1  -> 22.5
      2  -> 23.0
      ...
      12 -> 28.0 (top clamp)
    """
    if bucket <= 0:
        return 22.0
    if bucket >= 12:
        return 28.0
    return 22.5 + 0.5 * (bucket - 1)


def decode(data: bytes, base_ms: int, fast_ms: int):
    rows = []

    start = find_header(data)
    if start < 0:
        raise ValueError("Could not find compact stream header (C1 01). The file likely starts with text like 'T=' or '*timestamp*'.")

    # Parse header
    if start + 5 > len(data):
        raise ValueError("Truncated header.")
    magic, ver, count_lo, count_hi, bucket_cnt = data[start:start+5]
    if magic != 0xC1 or ver != 0x01:
        raise ValueError(f"Bad header at offset {start}: magic={magic:#x} ver={ver:#x}")
    if bucket_cnt != 13:
        sys.stderr.write(f"Warning: bucket_count={bucket_cnt}, expected 13.\n")
    off = start + 5

    # Expect initial OPC_TS
    if off >= len(data) or data[off] != OPC_TS:
        raise ValueError("Expected first timestamp (OPC_TS) after header; did you trigger the binary dumper?")
    off += 1
    start_ms, off = u32le(data, off)

    # State
    time_ms = start_ms
    in_excursion = False
    index = 0

    while off < len(data):
        b = data[off]
        off += 1

        if b == OPC_END:
            break
        if b == OPC_E:
            in_excursion = not in_excursion
            continue
        if b == OPC_TS:
            # If an extra timestamp appears, resync absolute time.
            time_ms, off = u32le(data, off)
            continue

        # Packed data: two 4-bit samples
        hi = (b >> 4) & 0x0F
        lo = b & 0x0F

        # Sample 1
        tm = bucket_to_temp_mid_c(hi)
        rows.append((index, time_ms, hi, tm, in_excursion))
        index += 1
        time_ms += fast_ms if in_excursion else base_ms

        # Sample 2
        tm = bucket_to_temp_mid_c(lo)
        rows.append((index, time_ms, lo, tm, in_excursion))
        index += 1
        time_ms += fast_ms if in_excursion else base_ms

    return rows

def write_csv(rows, path: str):
    header = "index,time_ms,bucket,temp_mid_C,in_excursion\n"
    if path == "-":
        sys.stdout.write(header)
        for i,t,b,tm,exc in rows:
            sys.stdout.write(f"{i},{t},{b},{tm:.2f},{int(exc)}\n")
    else:
        with open(path, "w", encoding="utf-8") as f:
            f.write(header)
            for i,t,b,tm,exc in rows:
                f.write(f"{i},{t},{b},{tm:.2f},{int(exc)}\n")

def find_all_headers(data: bytes):
    idx = 0
    while True:
        pos = data.find(b"\xC1\x01", idx)
        if pos < 0:
            return
        yield pos
        idx = pos + 2

def main():
    args = parse_args()
    raw = read_all(args.infile)

    seg_no = 0
    all_rows = []
    for start in find_all_headers(raw):
        seg_no += 1
        try:
            rows = decode(raw[start:], base_ms=args.base, fast_ms=args.fast)
            # tag rows with segment number
            rows = [(seg_no, *r) for r in rows]
            all_rows.extend(rows)
        except Exception as e:
            sys.stderr.write(f"Segment {seg_no} at offset {start}: {e}\n")
            continue

    if not all_rows:
        raise SystemExit("No decodable segments found.")

    # write CSV with a segment column
    header = "segment,index,time_ms,bucket,temp_mid_C,in_excursion\n"
    if args.csv == "-":
        sys.stdout.write(header)
        for seg,i,t,b,tm,exc in all_rows:
            sys.stdout.write(f"{seg},{i},{t},{b},{tm:.2f},{int(exc)}\n")
    else:
        with open(args.csv, "w", encoding="utf-8") as f:
            f.write(header)
            for seg,i,t,b,tm,exc in all_rows:
                f.write(f"{seg},{i},{t},{b},{tm:.2f},{int(exc)}\n")


if __name__ == "__main__":
    main()
