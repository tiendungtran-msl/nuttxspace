"""
UAV Ground Control Station - Protocol Layer

Định nghĩa packet structure tương ứng với MCU side.
Hỗ trợ 4 IMU riêng lẻ (raw data từ chip).
"""

import struct
from dataclasses import dataclass, field
from typing import Optional, Tuple, List

# Magic numbers
TELEM_MAGIC_START = 0x55AA
TELEM_MAGIC_END = 0xAA55
TELEM_PACKET_SIZE = 212     # Updated for 4 IMUs
TELEM_NUM_IMUS = 4

# Struct format strings (little-endian)
# Header: magic(H) + seq(H) + timestamp(I) = 8 bytes
HEADER_FMT = '<HHI'
HEADER_SIZE = struct.calcsize(HEADER_FMT)

# IMU: 7 floats = 28 bytes (per IMU)
IMU_FMT = '<7f'
IMU_SIZE = struct.calcsize(IMU_FMT)

# Mag: 3 floats = 12 bytes
MAG_FMT = '<3f'
MAG_SIZE = struct.calcsize(MAG_FMT)

# Baro: 2 floats = 8 bytes
BARO_FMT = '<2f'
BARO_SIZE = struct.calcsize(BARO_FMT)

# GPS: lat(i) + lon(i) + alt(i) + speed(I) + heading(h) + fix(B) + sats(B) + hdop(H) + vdop(H) = 24 bytes
GPS_FMT = '<iiiIhBBHH'
GPS_SIZE = struct.calcsize(GPS_FMT)

# Attitude: 8 floats = 32 bytes
ATTITUDE_FMT = '<8f'
ATTITUDE_SIZE = struct.calcsize(ATTITUDE_FMT)

# Status: health(B) + imus(B) + flags(B) + reserved(B) + cpu(H) + batt(H) + loop(I) = 12 bytes
STATUS_FMT = '<BBBBHHI'
STATUS_SIZE = struct.calcsize(STATUS_FMT)

# Footer: crc(H) + magic(H) = 4 bytes
FOOTER_FMT = '<HH'
FOOTER_SIZE = struct.calcsize(FOOTER_FMT)


@dataclass
class ImuData:
    """Data for a single IMU"""
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    temperature: float = 0.0


@dataclass
class TelemetryData:
    """Parsed telemetry packet data with 4 individual IMUs"""
    
    # Header
    sequence: int = 0
    timestamp_us: int = 0
    
    # 4 Individual IMUs (raw data from chips)
    imu: List[ImuData] = field(default_factory=lambda: [ImuData() for _ in range(TELEM_NUM_IMUS)])
    
    # Legacy single-IMU access (for backward compatibility - uses IMU 0)
    @property
    def gyro_x(self) -> float:
        return self.imu[0].gyro_x
    
    @property
    def gyro_y(self) -> float:
        return self.imu[0].gyro_y
    
    @property
    def gyro_z(self) -> float:
        return self.imu[0].gyro_z
    
    @property
    def accel_x(self) -> float:
        return self.imu[0].accel_x
    
    @property
    def accel_y(self) -> float:
        return self.imu[0].accel_y
    
    @property
    def accel_z(self) -> float:
        return self.imu[0].accel_z
    
    @property
    def imu_temp(self) -> float:
        return self.imu[0].temperature
    
    # Mag
    mag_x: float = 0.0
    mag_y: float = 0.0
    mag_z: float = 0.0
    
    # Baro
    pressure: float = 0.0
    baro_alt: float = 0.0
    
    # GPS
    latitude: float = 0.0      # degrees
    longitude: float = 0.0     # degrees
    altitude_msl: float = 0.0  # meters
    ground_speed: float = 0.0  # m/s
    heading: float = 0.0       # degrees
    fix_type: int = 0
    satellites: int = 0
    hdop: float = 0.0
    vdop: float = 0.0
    
    # Attitude
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    roll: float = 0.0          # rad
    pitch: float = 0.0         # rad
    yaw: float = 0.0           # rad
    innovation_var: float = 0.0
    
    # Status
    health_level: int = 0
    healthy_imus: int = 0
    sensor_flags: int = 0
    cpu_load: float = 0.0      # percent
    battery_mv: int = 0
    loop_count: int = 0
    
    # PC-side metadata
    pc_timestamp_ms: int = 0
    crc_valid: bool = True
    
    @property
    def roll_deg(self) -> float:
        """Roll in degrees"""
        import math
        return math.degrees(self.roll)
    
    @property
    def pitch_deg(self) -> float:
        """Pitch in degrees"""
        import math
        return math.degrees(self.pitch)
    
    @property
    def yaw_deg(self) -> float:
        """Yaw in degrees"""
        import math
        return math.degrees(self.yaw)
    
    @property
    def timestamp_sec(self) -> float:
        """Timestamp in seconds"""
        return self.timestamp_us / 1_000_000.0


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    """
    Calculate CRC16-CCITT (polynomial 0x1021)
    
    Phải khớp với implementation trên MCU.
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


def decode_packet(raw: bytes) -> Optional[TelemetryData]:
    """
    Decode 212-byte binary packet to TelemetryData.
    
    Returns None if packet is invalid.
    """
    if len(raw) != TELEM_PACKET_SIZE:
        return None
    
    # Parse header
    offset = 0
    magic, seq, timestamp = struct.unpack_from(HEADER_FMT, raw, offset)
    offset += HEADER_SIZE
    
    if magic != TELEM_MAGIC_START:
        return None
    
    # Parse 4 IMUs
    imu_list = []
    for i in range(TELEM_NUM_IMUS):
        imu_raw = struct.unpack_from(IMU_FMT, raw, offset)
        offset += IMU_SIZE
        imu_list.append(ImuData(
            gyro_x=imu_raw[0],
            gyro_y=imu_raw[1],
            gyro_z=imu_raw[2],
            accel_x=imu_raw[3],
            accel_y=imu_raw[4],
            accel_z=imu_raw[5],
            temperature=imu_raw[6]
        ))
    
    # Parse Mag
    mag = struct.unpack_from(MAG_FMT, raw, offset)
    offset += MAG_SIZE
    
    # Parse Baro
    baro = struct.unpack_from(BARO_FMT, raw, offset)
    offset += BARO_SIZE
    
    # Parse GPS
    gps = struct.unpack_from(GPS_FMT, raw, offset)
    offset += GPS_SIZE
    
    # Parse Attitude
    att = struct.unpack_from(ATTITUDE_FMT, raw, offset)
    offset += ATTITUDE_SIZE
    
    # Parse Status
    status = struct.unpack_from(STATUS_FMT, raw, offset)
    offset += STATUS_SIZE
    
    # Parse Footer
    crc_received, end_magic = struct.unpack_from(FOOTER_FMT, raw, offset)
    
    if end_magic != TELEM_MAGIC_END:
        return None
    
    # Validate CRC
    crc_data = raw[:TELEM_PACKET_SIZE - FOOTER_SIZE]
    crc_calculated = crc16_ccitt(crc_data)
    crc_valid = (crc_received == crc_calculated)
    
    # Build result
    data = TelemetryData(
        sequence=seq,
        timestamp_us=timestamp,
        
        imu=imu_list,
        
        mag_x=mag[0],
        mag_y=mag[1],
        mag_z=mag[2],
        
        pressure=baro[0],
        baro_alt=baro[1],
        
        latitude=gps[0] / 1e7,
        longitude=gps[1] / 1e7,
        altitude_msl=gps[2] / 1000.0,
        ground_speed=gps[3] / 100.0,
        heading=gps[4] / 100.0,
        fix_type=gps[5],
        satellites=gps[6],
        hdop=gps[7] / 100.0,
        vdop=gps[8] / 100.0,
        
        qw=att[0],
        qx=att[1],
        qy=att[2],
        qz=att[3],
        roll=att[4],
        pitch=att[5],
        yaw=att[6],
        innovation_var=att[7],
        
        health_level=status[0],
        healthy_imus=status[1],
        sensor_flags=status[2],
        cpu_load=status[4] / 10.0,
        battery_mv=status[5],
        loop_count=status[6],
        
        crc_valid=crc_valid
    )
    
    return data
