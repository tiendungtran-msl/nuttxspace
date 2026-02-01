#!/usr/bin/env python3
"""
Simple Telemetry Decoder Test

Chương trình đơn giản để kiểm tra đường truyền dữ liệu từ STM32.
Hiển thị raw bytes và decode packets.

Usage:
    python3 test_decode.py [port] [baudrate]
    
Example:
    python3 test_decode.py /dev/ttyUSB0 921600
"""

import sys
import time
import struct
import serial

# Constants from packet.py
TELEM_MAGIC_START = 0x55AA
TELEM_MAGIC_END = 0xAA55
TELEM_PACKET_SIZE = 128


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    """Calculate CRC16-CCITT"""
    crc = initial
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class SimpleDecoder:
    """Simple state machine decoder"""
    
    def __init__(self):
        self.buffer = bytearray()
        self.state = 'SYNC_1'
        self.packets_ok = 0
        self.packets_crc_fail = 0
        self.sync_errors = 0
        self.bytes_total = 0
    
    def feed(self, data: bytes):
        """Feed bytes and return list of valid packets"""
        packets = []
        self.bytes_total += len(data)
        
        for byte in data:
            pkt = self._process_byte(byte)
            if pkt:
                packets.append(pkt)
        
        return packets
    
    def _process_byte(self, byte: int):
        """Process single byte"""
        
        if self.state == 'SYNC_1':
            # Looking for 0x55 (first byte of 0x55AA in little-endian)
            if byte == 0xAA:
                self.buffer.clear()
                self.buffer.append(byte)
                self.state = 'SYNC_2'
            return None
        
        elif self.state == 'SYNC_2':
            # Looking for 0xAA (second byte of 0x55AA in little-endian)
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
                # Full packet - decode
                raw = bytes(self.buffer)
                self.buffer.clear()
                self.state = 'SYNC_1'
                return self._decode(raw)
            
            return None
    
    def _decode(self, raw: bytes):
        """Decode packet and validate"""
        # Check end magic
        end_magic = struct.unpack_from('<H', raw, TELEM_PACKET_SIZE - 2)[0]
        if end_magic != TELEM_MAGIC_END:
            self.sync_errors += 1
            return None
        
        # Check CRC
        crc_received = struct.unpack_from('<H', raw, TELEM_PACKET_SIZE - 4)[0]
        crc_data = raw[:TELEM_PACKET_SIZE - 4]
        crc_calc = crc16_ccitt(crc_data)
        
        if crc_received != crc_calc:
            self.packets_crc_fail += 1
            print(f"  CRC FAIL: recv=0x{crc_received:04X} calc=0x{crc_calc:04X}")
            return None
        
        self.packets_ok += 1
        
        # Parse key fields
        seq = struct.unpack_from('<H', raw, 2)[0]
        timestamp = struct.unpack_from('<I', raw, 4)[0]
        
        # IMU (offset 8)
        gyro_x, gyro_y, gyro_z = struct.unpack_from('<3f', raw, 8)
        accel_x, accel_y, accel_z = struct.unpack_from('<3f', raw, 20)
        
        # Attitude (offset 60)
        qw, qx, qy, qz = struct.unpack_from('<4f', raw, 60)
        roll, pitch, yaw = struct.unpack_from('<3f', raw, 76)
        
        return {
            'seq': seq,
            'timestamp_us': timestamp,
            'gyro': (gyro_x, gyro_y, gyro_z),
            'accel': (accel_x, accel_y, accel_z),
            'quat': (qw, qx, qy, qz),
            'rpy_rad': (roll, pitch, yaw),
            'raw': raw
        }


def print_hex_dump(data: bytes, per_line: int = 16):
    """Print hex dump of data"""
    for i in range(0, len(data), per_line):
        chunk = data[i:i+per_line]
        hex_str = ' '.join(f'{b:02X}' for b in chunk)
        ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        print(f"  {i:04X}: {hex_str:<{per_line*3}}  {ascii_str}")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 921600
    
    print(f"=== Telemetry Decoder Test ===")
    print(f"Port: {port} @ {baudrate} baud")
    print(f"Expected packet size: {TELEM_PACKET_SIZE} bytes")
    print(f"Magic start: 0x{TELEM_MAGIC_START:04X} (little-endian: AA 55)")
    print(f"Magic end: 0x{TELEM_MAGIC_END:04X} (little-endian: 55 AA)")
    print(f"Press Ctrl+C to stop\n")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        print(f"Connected to {port}")
    except Exception as e:
        print(f"Failed to open {port}: {e}")
        return 1
    
    decoder = SimpleDecoder()
    start_time = time.time()
    last_print = start_time
    last_seq = -1
    seq_drops = 0
    
    print("\n--- Waiting for data ---\n")
    
    # Show raw bytes first
    raw_phase = True
    raw_bytes = bytearray()
    
    try:
        while True:
            data = ser.read(256)
            if not data:
                continue
            
            # Raw phase - show first 512 bytes received
            if raw_phase:
                raw_bytes.extend(data)
                if len(raw_bytes) >= 512:
                    print("=== First 512 bytes received (hex dump) ===")
                    print_hex_dump(bytes(raw_bytes[:512]))
                    
                    # Check for magic bytes
                    magic_found = False
                    for i in range(len(raw_bytes) - 1):
                        if raw_bytes[i] == 0xAA and raw_bytes[i+1] == 0x55:
                            print(f"\n✓ Found magic 0x55AA at offset {i}")
                            magic_found = True
                            break
                    
                    if not magic_found:
                        print("\n✗ No magic bytes 0xAA 0x55 found in first 512 bytes!")
                        print("  This suggests console text or wrong UART")
                    
                    print("\n--- Now decoding packets ---\n")
                    raw_phase = False
            
            # Decode
            packets = decoder.feed(data)
            
            for pkt in packets:
                # Check sequence
                if last_seq >= 0:
                    expected = (last_seq + 1) & 0xFFFF
                    if pkt['seq'] != expected:
                        gap = (pkt['seq'] - expected) & 0xFFFF
                        seq_drops += gap
                last_seq = pkt['seq']
                
                # Print packet info
                import math
                roll_deg = math.degrees(pkt['rpy_rad'][0])
                pitch_deg = math.degrees(pkt['rpy_rad'][1])
                yaw_deg = math.degrees(pkt['rpy_rad'][2])
                
                print(f"[{pkt['seq']:5d}] t={pkt['timestamp_us']/1e6:.3f}s "
                      f"| R:{roll_deg:+6.1f}° P:{pitch_deg:+6.1f}° Y:{yaw_deg:+6.1f}° "
                      f"| Ax:{pkt['accel'][0]:+5.2f} Ay:{pkt['accel'][1]:+5.2f} Az:{pkt['accel'][2]:+5.2f}")
            
            # Stats every 2 seconds
            now = time.time()
            if now - last_print >= 2.0:
                elapsed = now - start_time
                rate = decoder.packets_ok / elapsed if elapsed > 0 else 0
                bps = decoder.bytes_total / elapsed if elapsed > 0 else 0
                
                print(f"\n--- Stats: {decoder.packets_ok} pkts OK, "
                      f"{decoder.packets_crc_fail} CRC fail, "
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
        print(f"CRC failures: {decoder.packets_crc_fail}")
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
