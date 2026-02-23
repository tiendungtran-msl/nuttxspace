#!/usr/bin/env python3
"""
Telemetry Decoder Test

Kiểm tra đường truyền dữ liệu từ STM32.
Sử dụng decoder chung từ uav_gcs/protocol.

Usage:
    python3 test_decode.py [port] [baudrate]

Example:
    python3 test_decode.py /dev/ttyUSB1 921600
"""

import sys
import os
import time
import math

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import serial

# Import từ protocol module (single source of truth)
from uav_gcs.protocol.packet import (
    TELEM_PACKET_SIZE, TELEM_NUM_IMUS,
    TELEM_MAGIC_START, TELEM_MAGIC_END,
    decode_packet, crc16_ccitt
)


class PacketDecoder:
    """State machine decoder for telemetry packets"""

    def __init__(self):
        self.buffer = bytearray()
        self.state = 'SYNC_1'
        self.packets_ok = 0
        self.crc_fail = 0
        self.sync_errors = 0
        self.bytes_total = 0

    def feed(self, data: bytes):
        """Feed bytes, return list of decoded TelemetryData"""
        packets = []
        self.bytes_total += len(data)

        for byte in data:
            pkt = self._process_byte(byte)
            if pkt is not None:
                packets.append(pkt)

        return packets

    def _process_byte(self, byte: int):
        if self.state == 'SYNC_1':
            # 0x55AA little-endian -> first byte on wire is 0xAA
            if byte == 0xAA:
                self.buffer.clear()
                self.buffer.append(byte)
                self.state = 'SYNC_2'
            return None

        elif self.state == 'SYNC_2':
            if byte == 0x55:
                self.buffer.append(byte)
                self.state = 'PAYLOAD'
            else:
                self.sync_errors += 1
                if byte == 0xAA:
                    self.buffer.clear()
                    self.buffer.append(byte)
                else:
                    self.state = 'SYNC_1'
            return None

        elif self.state == 'PAYLOAD':
            self.buffer.append(byte)

            if len(self.buffer) >= TELEM_PACKET_SIZE:
                raw = bytes(self.buffer)
                self.buffer.clear()
                self.state = 'SYNC_1'
                return self._decode(raw)

            return None

    def _decode(self, raw: bytes):
        data = decode_packet(raw)

        if data is None:
            self.sync_errors += 1
            return None

        if not data.crc_valid:
            self.crc_fail += 1
            print(f"  CRC FAIL: packet discarded")
            return None

        self.packets_ok += 1
        return data


def print_hex_dump(data: bytes, per_line: int = 16):
    """Print hex dump"""
    for i in range(0, len(data), per_line):
        chunk = data[i:i+per_line]
        hex_str = ' '.join(f'{b:02X}' for b in chunk)
        ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        print(f"  {i:04X}: {hex_str:<{per_line*3}}  {ascii_str}")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB1'
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 921600

    print("=== Telemetry Decoder Test ===")
    print(f"Port: {port} @ {baudrate} baud")
    print(f"Packet size: {TELEM_PACKET_SIZE} bytes")
    print(f"IMUs: {TELEM_NUM_IMUS}")
    print(f"Magic start: 0x{TELEM_MAGIC_START:04X} (wire: AA 55)")
    print(f"Magic end: 0x{TELEM_MAGIC_END:04X} (wire: 55 AA)")
    print("Press Ctrl+C to stop\n")

    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        print(f"Connected to {port}\n")
    except Exception as e:
        print(f"ERROR: Failed to open {port}: {e}")
        return 1

    decoder = PacketDecoder()
    start_time = time.time()
    last_print = start_time
    last_seq = -1
    seq_drops = 0

    # Raw phase: show first 512 bytes
    raw_phase = True
    raw_bytes = bytearray()

    try:
        while True:
            data = ser.read(256)
            if not data:
                continue

            # Show first bytes for diagnosis
            if raw_phase:
                raw_bytes.extend(data)
                if len(raw_bytes) >= 512:
                    print("=== First 512 bytes (hex dump) ===")
                    print_hex_dump(bytes(raw_bytes[:512]))

                    # Check for magic
                    found = False
                    for i in range(len(raw_bytes) - 1):
                        if raw_bytes[i] == 0xAA and raw_bytes[i+1] == 0x55:
                            print(f"\n✓ Magic 0x55AA found at offset {i}")
                            found = True
                            break

                    if not found:
                        print("\n✗ No magic 0xAA 0x55 found - wrong port/baud or console text?")

                    print("\n--- Decoding packets ---\n")
                    raw_phase = False

            # Decode
            packets = decoder.feed(data)

            for pkt in packets:
                # Check sequence
                if last_seq >= 0:
                    expected = (last_seq + 1) & 0xFFFF
                    if pkt.sequence != expected:
                        gap = (pkt.sequence - expected) & 0xFFFF
                        seq_drops += gap
                last_seq = pkt.sequence

                # Print compact info
                r_deg = math.degrees(pkt.roll)
                p_deg = math.degrees(pkt.pitch)
                y_deg = math.degrees(pkt.yaw)

                # Show IMU0 accel as sanity check
                imu0 = pkt.imu[0]
                print(f"[{pkt.sequence:5d}] t={pkt.timestamp_sec:.3f}s "
                      f"| R:{r_deg:+6.1f}° P:{p_deg:+6.1f}° Y:{y_deg:+6.1f}° "
                      f"| Az:{imu0.accel_z:+5.2f} T:{imu0.temperature:.1f}°C")

            # Stats every 2 seconds
            now = time.time()
            if now - last_print >= 2.0:
                elapsed = now - start_time
                rate = decoder.packets_ok / elapsed if elapsed > 0 else 0
                bps = decoder.bytes_total / elapsed if elapsed > 0 else 0

                print(f"\n--- Stats: {decoder.packets_ok} OK, "
                      f"{decoder.crc_fail} CRC fail, "
                      f"{decoder.sync_errors} sync err, "
                      f"{seq_drops} drops | "
                      f"{rate:.1f} pkt/s, {bps/1024:.1f} KB/s ---\n")
                last_print = now

    except KeyboardInterrupt:
        print("\n\n=== Final Stats ===")
        elapsed = time.time() - start_time
        print(f"Duration: {elapsed:.1f} seconds")
        print(f"Bytes received: {decoder.bytes_total}")
        print(f"Packets OK: {decoder.packets_ok}")
        print(f"CRC failures: {decoder.crc_fail}")
        print(f"Sync errors: {decoder.sync_errors}")
        print(f"Sequence drops: {seq_drops}")
        if elapsed > 0:
            print(f"Rate: {decoder.packets_ok/elapsed:.1f} packets/sec")
            print(f"Throughput: {decoder.bytes_total/elapsed/1024:.2f} KB/s")

    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
