#!/usr/bin/env python3
"""Dump raw hex data from serial port"""
import serial
import sys

PORT = "/dev/ttyUSB1"
BAUD = 921600

ser = serial.Serial(PORT, BAUD, timeout=0.1)
print(f"Listening on {PORT} @ {BAUD}...")

try:
    while True:
        data = ser.read(64)
        if data:
            # Print hex
            hex_str = ' '.join(f'{b:02X}' for b in data)
            # Print ASCII (printable chars only)
            ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in data)
            print(f"HEX: {hex_str}")
            print(f"ASC: {ascii_str}")
            print()
except KeyboardInterrupt:
    pass
finally:
    ser.close()
