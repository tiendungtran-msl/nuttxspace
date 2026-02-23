#!/usr/bin/env python3
"""
GPS UART Test Script

Đọc dữ liệu NMEA từ GPS qua UART và in thông tin GPS hiện tại.

Usage:
    python3 test_gps_uart.py --port /dev/ttyUSB1 --baud 115200
    python3 test_gps_uart.py -p /dev/ttyUSB1 -b 9600 --raw
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional

import serial


@dataclass
class GPSState:
    fix_type: int = 0
    satellites: int = 0
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    altitude_m: Optional[float] = None
    speed_mps: Optional[float] = None
    course_deg: Optional[float] = None
    hdop: Optional[float] = None
    vdop: Optional[float] = None
    utc_time: Optional[str] = None
    last_sentence: str = ""


def safe_float(text: str) -> Optional[float]:
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def safe_int(text: str) -> Optional[int]:
    if not text:
        return None
    try:
        return int(text)
    except ValueError:
        return None


def parse_latlon(value: str, hemi: str) -> Optional[float]:
    if not value or not hemi:
        return None

    try:
        raw = float(value)
    except ValueError:
        return None

    deg = int(raw / 100)
    minutes = raw - (deg * 100)
    decimal = deg + (minutes / 60.0)

    if hemi in ("S", "W"):
        decimal = -decimal

    return decimal


def nmea_checksum_ok(line: str) -> bool:
    if not line.startswith("$") or "*" not in line:
        return False

    body, checksum_text = line[1:].split("*", 1)
    checksum_text = checksum_text.strip()
    if len(checksum_text) < 2:
        return False

    calc = 0
    for c in body:
        calc ^= ord(c)

    try:
        expected = int(checksum_text[:2], 16)
    except ValueError:
        return False

    return calc == expected


def parse_utc_time(time_text: str, date_text: Optional[str] = None) -> Optional[str]:
    if not time_text:
        return None

    if len(time_text) < 6:
        return None

    hh = time_text[0:2]
    mm = time_text[2:4]
    ss = time_text[4:6]

    try:
        hour = int(hh)
        minute = int(mm)
        second = int(ss)
    except ValueError:
        return None

    if date_text and len(date_text) == 6:
        try:
            day = int(date_text[0:2])
            month = int(date_text[2:4])
            year = int(date_text[4:6]) + 2000
            dt = datetime(year, month, day, hour, minute, second, tzinfo=timezone.utc)
            return dt.strftime("%Y-%m-%d %H:%M:%S UTC")
        except ValueError:
            return None

    return f"{hour:02d}:{minute:02d}:{second:02d} UTC"


def update_from_sentence(line: str, state: GPSState) -> bool:
    if not line.startswith("$"):
        return False

    if not nmea_checksum_ok(line):
        return False

    body = line[1:line.find("*")]
    fields = body.split(",")
    if len(fields) < 1:
        return False

    sentence_type = fields[0]
    state.last_sentence = sentence_type

    updated = False

    if sentence_type.endswith("GGA"):
        if len(fields) >= 10:
            state.utc_time = parse_utc_time(fields[1]) or state.utc_time

            lat = parse_latlon(fields[2], fields[3])
            lon = parse_latlon(fields[4], fields[5])
            if lat is not None and lon is not None:
                state.latitude = lat
                state.longitude = lon
                updated = True

            fix = safe_int(fields[6])
            if fix is not None:
                state.fix_type = fix
                updated = True

            sats = safe_int(fields[7])
            if sats is not None:
                state.satellites = sats
                updated = True

            hdop = safe_float(fields[8])
            if hdop is not None:
                state.hdop = hdop
                updated = True

            alt = safe_float(fields[9])
            if alt is not None:
                state.altitude_m = alt
                updated = True

    elif sentence_type.endswith("RMC"):
        if len(fields) >= 10:
            state.utc_time = parse_utc_time(fields[1], fields[9]) or state.utc_time

            status = fields[2] if len(fields) > 2 else ""
            if status == "A":
                lat = parse_latlon(fields[3], fields[4])
                lon = parse_latlon(fields[5], fields[6])
                if lat is not None and lon is not None:
                    state.latitude = lat
                    state.longitude = lon
                    updated = True

            speed_knots = safe_float(fields[7])
            if speed_knots is not None:
                state.speed_mps = speed_knots * 0.514444
                updated = True

            course = safe_float(fields[8])
            if course is not None:
                state.course_deg = course
                updated = True

            if status == "A":
                state.fix_type = max(state.fix_type, 2)
                updated = True

    elif sentence_type.endswith("GSA"):
        if len(fields) >= 18:
            fix = safe_int(fields[2])
            if fix is not None:
                state.fix_type = max(state.fix_type, fix)
                updated = True

            pdop = safe_float(fields[15])
            hdop = safe_float(fields[16])
            vdop = safe_float(fields[17])

            if hdop is not None:
                state.hdop = hdop
                updated = True
            if vdop is not None:
                state.vdop = vdop
                updated = True
            if pdop is not None and state.hdop is None:
                state.hdop = pdop
                updated = True

    return updated


def print_gps_info(state: GPSState) -> None:
    fix_name = {
        0: "NO FIX",
        1: "GPS FIX",
        2: "2D FIX",
        3: "3D FIX",
    }.get(state.fix_type, f"FIX {state.fix_type}")

    lat_text = f"{state.latitude:.7f}" if state.latitude is not None else "--"
    lon_text = f"{state.longitude:.7f}" if state.longitude is not None else "--"
    alt_text = f"{state.altitude_m:.2f} m" if state.altitude_m is not None else "--"
    spd_text = f"{state.speed_mps:.2f} m/s" if state.speed_mps is not None else "--"
    crs_text = f"{state.course_deg:.1f} deg" if state.course_deg is not None else "--"
    hdop_text = f"{state.hdop:.2f}" if state.hdop is not None else "--"
    vdop_text = f"{state.vdop:.2f}" if state.vdop is not None else "--"
    utc_text = state.utc_time if state.utc_time is not None else "--"

    print("\n=== GPS Current Info ===")
    print(f"Fix:        {fix_name}")
    print(f"Satellites: {state.satellites}")
    print(f"Latitude:   {lat_text}")
    print(f"Longitude:  {lon_text}")
    print(f"Altitude:   {alt_text}")
    print(f"Speed:      {spd_text}")
    print(f"Course:     {crs_text}")
    print(f"HDOP/VDOP:  {hdop_text} / {vdop_text}")
    print(f"UTC Time:   {utc_text}")
    print(f"Source:     {state.last_sentence}")
    print("========================")


def run(port: str, baud: int, timeout: float, print_period: float, raw: bool) -> int:
    stop = False

    def _handle_signal(_sig, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
    except Exception as exc:
        print(f"[ERROR] Cannot open {port}: {exc}")
        return 1

    print("=== GPS UART Test ===")
    print(f"Port: {port} @ {baud}")
    print("Press Ctrl+C to stop")

    state = GPSState()
    line_buf = bytearray()
    last_print = 0.0
    valid_lines = 0
    checksum_fail = 0

    try:
        while not stop:
            data = ser.read(256)
            if not data:
                now = time.time()
                if now - last_print >= print_period:
                    print_gps_info(state)
                    last_print = now
                continue

            for byte in data:
                if byte in (0x0A, 0x0D):
                    if not line_buf:
                        continue

                    try:
                        line = line_buf.decode("ascii", errors="ignore").strip()
                    finally:
                        line_buf.clear()

                    if not line:
                        continue

                    if raw:
                        print(line)

                    if line.startswith("$") and "*" in line:
                        if update_from_sentence(line, state):
                            valid_lines += 1
                        else:
                            if not nmea_checksum_ok(line):
                                checksum_fail += 1

                    now = time.time()
                    if now - last_print >= print_period:
                        print_gps_info(state)
                        print(f"Lines OK: {valid_lines}, checksum fail: {checksum_fail}")
                        last_print = now
                else:
                    if len(line_buf) < 1024:
                        line_buf.append(byte)
                    else:
                        line_buf.clear()

    finally:
        ser.close()
        print("\n[INFO] Serial closed")

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Test nhận dữ liệu GPS từ UART")
    parser.add_argument("-p", "--port", default="/dev/ttyUSB1", help="Serial port (default: /dev/ttyUSB1)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("-t", "--timeout", type=float, default=0.1, help="Serial timeout seconds")
    parser.add_argument("--period", type=float, default=1.0, help="Chu kỳ in GPS info (s)")
    parser.add_argument("--raw", action="store_true", help="In thêm raw NMEA lines")

    args = parser.parse_args()
    return run(args.port, args.baud, args.timeout, args.period, args.raw)


if __name__ == "__main__":
    sys.exit(main())
