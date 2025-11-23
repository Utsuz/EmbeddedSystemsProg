#!/usr/bin/env python3
"""
live_decode.py

Reads from Master Pico's USB serial, automatically:
1. Waits for a DUMP_START log line (GP21 trigger).
2. Simultaneously captures the time log.
3. Reads the compact binary dump using the size from DUMP_START.
4. Decodes it using the same timing as the firmware.
5. Writes decoded.csv with real datetime stamps.
"""

import serial
import time
import re
import binascii
from datetime import datetime, timedelta
import struct  # Added import for potential future use or consistency

# =============================
# USER CONFIG
# =============================

# PORT = "COM4"          # CHANGE THIS to your Master Pico USB port
PORT = "/dev/ttyACM5"    # Linux Port
BAUD = 115200
OUT_CSV = "decoded.csv"

# Must match your firmware:
BASE_MS = 5000         # normal sampling period (ms)
FAST_MS = 200          # excursion sampling period (ms)

# Opcodes (must match C cod2)
OPC_TS  = 0xF0
OPC_E   = 0xE5
OPC_END = 0xEE

# Binary Dump Header and Time CMDs
CMD_DUMP_HDR = 0x50
BINARY_DUMP_HEADER_SIZE = 4 # CMD + 2B Length + DONE_CHAR (0x06)

CMD_DUMP_TIME = 0x51
BINARY_TIME_BLOCK_SIZE = 9  # CMD + 8B Unix Time

# =============================
# DECODE HELPERS
# =============================

def bucket_to_temp_mid_c(bucket: int) -> float:
    """Convert bucket index to approximate temperature midpoint."""
    if bucket <= 0:
        return 22.0
    if bucket >= 12:
        return 28.0
    return 22.5 + 0.5 * (bucket - 1)


def decode_compact(data: bytes):
    """
    Decode compact-byte stream into rows:
    (segment, index, time_ms, bucket, temp_mid_C, in_excursion)
    """
    rows = []
    segments = []

    # Find all timestamps (each marks start of a segment)
    for i in range(len(data)):
        if data[i] == OPC_TS:
            segments.append(i)

    seg_no = 0
    for ts_off in segments:
        seg_no += 1
        off = ts_off + 1

        # Read initial timestamp (uint32 LE)
        if off + 4 > len(data):
            continue
        t_ms = (
            data[off]
            | (data[off + 1] << 8)
            | (data[off + 2] << 16)
            | (data[off + 3] << 24)
        )
        off += 4

        time_ms = t_ms
        in_exc = False
        index = 0

        while off < len(data):
            b = data[off]
            off += 1

            if b == OPC_END:
                # End of whole stream
                break
            if b == OPC_E:
                # Toggle excursion
                in_exc = not in_exc
                continue
            if b == OPC_TS:
                # New timestamp chunk
                if off + 4 > len(data):
                    break
                time_ms = (
                    data[off]
                    | (data[off + 1] << 8)
                    | (data[off + 2] << 16)
                    | (data[off + 3] << 24)
                )
                off += 4
                continue

            # Normal data byte: two nibbles (high then low)
            hi = (b >> 4) & 0x0F
            lo = b & 0x0F

            # High nibble sample
            tm = bucket_to_temp_mid_c(hi)
            rows.append((seg_no, index, time_ms, hi, tm, in_exc))
            index += 1
            time_ms += FAST_MS if in_exc else BASE_MS

            # Low nibble sample
            tm = bucket_to_temp_mid_c(lo)
            rows.append((seg_no, index, time_ms, lo, tm, in_exc))
            index += 1
            time_ms += FAST_MS if in_exc else BASE_MS

    return rows


# =============================
# TIME PARSING
# =============================

def parse_time_line(line: str):
    """
    Try to parse a datetime from:
    - "TIME YYYY-MM-DD HH:MM:SS"
    - "[Master] Sent current NTP time: Sun Nov 23 00:07:16 2025"
    - "[Master] Extracted start time from Slave data: Sun Nov 23 13:21:49 2025" (NEW)
    Returns datetime or None.
    """

    line = line.strip()

    # Case 1: simple TIME line
    if line.startswith("TIME "):
        # TIME 2025-11-23 00:07:16
        m = re.match(r"TIME\s+(\d{4}-\d{2}-\d{2})\s+(\d{2}:\d{2}:\d{2})", line)
        if m:
            dt_str = m.group(1) + " " + m.group(2)
            try:
                return datetime.strptime(dt_str, "%Y-%m-%d %H:%M:%S")
            except ValueError:
                return None

    # Case 2: old NTP log format:
    # [Master] Sent current NTP time: Sun Nov 23 00:07:16 2025
    if "Sent current NTP time:" in line:
        part = line.split("Sent current NTP time:")[1].strip()
        # Example: Sun Nov 23 00:07:16 2025
        try:
            return datetime.strptime(part, "%a %b %d %H:%M:%S %Y")
        except ValueError:
            return None

    # Case 3: NEW Extracted time log from Master:
    # [Master] Extracted start time from Slave data: Sun Nov 23 13:21:49 2025
    if "Extracted start time from Slave data:" in line:
        part = line.split("Extracted start time from Slave data:")[1].strip()
        try:
            return datetime.strptime(part, "%a %b %d %H:%M:%S %Y")
        except ValueError:
            return None

    return None

# =============================
# MAIN
# =============================

def main():
    print("========================================")
    print(" Live Compact Decoder (Master Pico USB)")
    print("========================================")
    print(f"Opening serial port {PORT} @ {BAUD}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return
        
    time.sleep(2)

    start_dt = None
    raw = bytearray()
    binary_size = 0 # NEW: Variable to hold the expected size

    print("\nWaiting for Binary DUMP Header (GP21) or Time Sync...")

    # Loop to capture both time log and binary size marker
    while binary_size == 0:
        
        # AGGRESSIVE ASCII CLEAR: Read and print *any* available line-by-line data 
        # to clear out buffered logs before checking for the binary header.
        while ser.in_waiting > 0:
            line_bytes = ser.readline()
            if line_bytes:
                try:
                    print("[SERIAL]", line_bytes.decode(errors="ignore").strip())
                except UnicodeDecodeError:
                    pass
            else:
                 # If readline returns nothing but in_waiting > 0, we rely on the binary check
                 break

        # 2. Check if the fixed-size Binary Header is available
        if ser.in_waiting >= BINARY_DUMP_HEADER_SIZE:
            header_bytes = ser.read(BINARY_DUMP_HEADER_SIZE)
            
            # Check the header structure: CMD_DUMP_HDR and DONE_CHAR tail
            if header_bytes[0] == CMD_DUMP_HDR and header_bytes[3] == 0x06:
                # Extract Length (Total Payload Size) - Little-Endian: LSB | MSB
                length = header_bytes[1] | (header_bytes[2] << 8)
                
                if length > 0:
                    binary_size = length
                    print(f"Detected Binary DUMP Header (0x{CMD_DUMP_HDR:02X}). Size: {binary_size} bytes.")
                    break
        
        time.sleep(0.01) # Avoid busy looping

    if binary_size == 0:
        print("ERROR: Did not receive valid Binary DUMP Header. Aborting.")
        return

    # --- 2. WAIT FOR AND READ BINARY TIME BLOCK (9 Bytes) ---
    print("Waiting for Binary Time Block (9 bytes)...")
    
    # Ensure 9 bytes are available (the Master sends this immediately after the header)
    while ser.in_waiting < BINARY_TIME_BLOCK_SIZE:
        # Read and print any residual ASCII logs while waiting
        line_bytes = ser.readline()
        if line_bytes:
            try:
                print("[SERIAL]", line_bytes.decode(errors="ignore").strip())
            except UnicodeDecodeError:
                pass
        time.sleep(0.01)

    time_block = ser.read(BINARY_TIME_BLOCK_SIZE)

    if time_block[0] == CMD_DUMP_TIME:
        # Extract 8-byte Unix time (Little-Endian 'Q' format)
        unix_time_bytes = time_block[1:]
        # Use struct.unpack for 64-bit unsigned integer ('Q') little-endian ('<')
        unix_timestamp = struct.unpack('<Q', unix_time_bytes)[0]
        
        start_dt = datetime.fromtimestamp(unix_timestamp)
        print(f"Extracted binary start datetime: {start_dt}")
    else:
        print(f"ERROR: Did not find expected Binary Time Command (0x{CMD_DUMP_TIME:02X}). Aborting.")
        return

    # --- 3. READ RAW COMPACT DATA ---
    print(f"Attempting to read {binary_size} raw data bytes...")
    
    # Wait until enough bytes are available
    while ser.in_waiting < binary_size:
        # Consume any remaining ASCII logs
        line_bytes = ser.readline()
        if line_bytes:
            try:
                print("[SERIAL]", line_bytes.decode(errors="ignore").strip())
            except UnicodeDecodeError:
                pass
        time.sleep(0.01)

    # Read the data and perform the initial checks *outside* the loop
    raw = ser.read(binary_size)

    # Note: The logic below this point was incorrectly nested under an 'else' block
    # in the original snippet, which is now corrected to be at the main level.

    print(f"\nTotal bytes received: {len(raw)}")
    print("First up to 32 bytes (hex):", binascii.hexlify(raw[:32]).decode())
    
    # Now consume the DUMP_END and any residual logs
    print("Consuming DUMP_END marker and remaining logs...")
    while True:
        line_bytes = ser.readline() 
        if not line_bytes:
            break
        try:
            line = line_bytes.decode(errors="ignore").strip()
            if line:
                print("[SERIAL]", line)
        except UnicodeDecodeError:
            continue
    
    # Final check for time before decoding
    if start_dt is None:
        # This check is now mostly redundant since we rely on the binary time block,
        # but kept for robustness.
        print("\nFATAL WARNING: Binary data acquired, but time sync is missing. CSV timestamps will be blank.")
        
    # Decode
    rows = decode_compact(bytes(raw))
    print(f"Decoded {len(rows)} samples.")

    # Write CSV
    with open(OUT_CSV, "w", encoding="utf-8", newline="") as f:
        f.write("segment,index,time_ms,real_datetime,bucket,temp_mid_C,in_excursion\n")
        for seg, idx, t_ms, bucket, temp_c, in_exc in rows:
            if start_dt is not None:
                dt = start_dt + timedelta(milliseconds=t_ms)
                dt_str = dt.strftime("%Y-%m-%d %H:%M:%S")
            else:
                dt_str = ""
            f.write(f"{seg},{idx},{t_ms},{dt_str},{bucket},{temp_c:.2f},{int(in_exc)}\n")

    print(f"\nCSV saved to: {OUT_CSV}")
    print("Done.")

if __name__ == "__main__":
    main()