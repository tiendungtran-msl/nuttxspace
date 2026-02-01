#!/usr/bin/env python3
"""
Quick test script to debug telemetry packet reception.
"""

import serial
import sys
import time

# Config
PORT = "/dev/ttyUSB1" if len(sys.argv) < 2 else sys.argv[1]
BAUD = 921600 if len(sys.argv) < 3 else int(sys.argv[2])

TELEM_MAGIC_START = 0x55AA
TELEM_MAGIC_END = 0xAA55
TELEM_PACKET_SIZE = 128


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    """
    CRC16-CCITT (polynomial 0x1021) - must match MCU implementation
    """
    crc = initial
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def crc16_ccitt_table(data: bytes, initial: int = 0xFFFF) -> int:
    """
    CRC16-CCITT using same algorithm as MCU (table-based)
    """
    # Same table as in telemetry_packet.c
    table = [
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
        0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
        0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
        0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
        0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
        0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
        0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
        0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
        0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
        0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
        0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
        0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
        0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
        0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
        0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
        0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
        0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
        0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
        0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
        0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
    ]
    
    crc = initial
    for byte in data:
        crc = ((crc << 8) ^ table[((crc >> 8) ^ byte) & 0xFF]) & 0xFFFF
    return crc


def main():
    print(f"Opening {PORT} @ {BAUD} baud...")
    
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0
        )
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        return 1
    
    print("Connected! Waiting for data...")
    print("=" * 60)
    
    # Stats
    bytes_received = 0
    packets_found = 0
    packets_valid = 0
    sync_errors = 0
    crc_errors = 0
    
    buffer = bytearray()
    start_time = time.time()
    
    try:
        while True:
            # Read available data
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                bytes_received += len(data)
                buffer.extend(data)
                
                # Print raw bytes (first 100)
                if bytes_received <= 500:
                    print(f"Raw ({len(data)} bytes): {data[:50].hex()}")
            
            # Try to find packets in buffer
            while len(buffer) >= 2:
                # Look for magic start (0x55 0xAA)
                if buffer[0] == 0x55 and buffer[1] == 0xAA:
                    if len(buffer) >= TELEM_PACKET_SIZE:
                        # Extract packet
                        raw = bytes(buffer[:TELEM_PACKET_SIZE])
                        buffer = buffer[TELEM_PACKET_SIZE:]
                        packets_found += 1
                        
                        # Check end magic
                        end_magic = (raw[127] << 8) | raw[126]
                        if end_magic != TELEM_MAGIC_END:
                            print(f"  Bad end magic: 0x{end_magic:04X}")
                            sync_errors += 1
                            continue
                        
                        # Check CRC
                        crc_received = (raw[125] << 8) | raw[124]
                        crc_data = raw[:124]
                        crc_calc_py = crc16_ccitt(crc_data)
                        crc_calc_tbl = crc16_ccitt_table(crc_data)
                        
                        if crc_received == crc_calc_tbl:
                            packets_valid += 1
                            
                            # Parse some fields
                            import struct
                            seq = struct.unpack_from('<H', raw, 2)[0]
                            ts = struct.unpack_from('<I', raw, 4)[0]
                            gyro = struct.unpack_from('<3f', raw, 8)
                            accel = struct.unpack_from('<3f', raw, 20)
                            
                            print(f"[PKT #{seq}] ts={ts/1e6:.3f}s  "
                                  f"gyro=({gyro[0]:.4f}, {gyro[1]:.4f}, {gyro[2]:.4f})  "
                                  f"accel=({accel[0]:.2f}, {accel[1]:.2f}, {accel[2]:.2f})")
                        else:
                            crc_errors += 1
                            print(f"  CRC mismatch: recv=0x{crc_received:04X} "
                                  f"calc_py=0x{crc_calc_py:04X} "
                                  f"calc_tbl=0x{crc_calc_tbl:04X}")
                    else:
                        break  # Need more data
                
                elif buffer[0] == 0x55:
                    # Could be start of magic, wait for more
                    if len(buffer) < 2:
                        break
                    else:
                        # Not magic, skip
                        sync_errors += 1
                        buffer.pop(0)
                else:
                    # Not 0x55, skip
                    buffer.pop(0)
            
            # Print stats every 2 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 2 == 0 and elapsed > 0.1:
                rate = bytes_received / elapsed
                print(f"\n[STATS] {elapsed:.1f}s: {bytes_received} bytes "
                      f"({rate:.0f} B/s), {packets_found} pkts found, "
                      f"{packets_valid} valid, {sync_errors} sync_err, {crc_errors} crc_err\n")
            
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        ser.close()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
