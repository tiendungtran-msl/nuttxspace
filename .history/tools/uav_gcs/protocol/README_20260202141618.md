# UAV GCS - Protocol Layer

## 📋 Tổng Quan

Module `protocol` xử lý tất cả việc giao tiếp với UAV qua UART. Đây là **lớp thấp nhất** trong kiến trúc GCS, chịu trách nhiệm:

1. **Nhận bytes** từ serial port (non-blocking)
2. **Sync** và phát hiện packet boundaries
3. **Decode** binary data thành Python objects
4. **Validate** CRC16 để đảm bảo tính toàn vẹn

### Vị Trí Trong Kiến Trúc

```
┌─────────────────────────────────────────────────────────────────┐
│                         GUI Layer                                │
│    (FastMainWindow, ControlPanel, SensorDisplay, ...)           │
└────────────────────────────────┬────────────────────────────────┘
                                 │ TelemetryData objects
                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│                        Data Layer                                │
│              (DataManager, RingBuffer, Logger)                   │
└────────────────────────────────┬────────────────────────────────┘
                                 │ TelemetryData objects
                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│                     PROTOCOL LAYER (module này)                  │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐        │
│  │    packet    │   │serial_receiver│   │  simulator   │        │
│  │  (decode)    │   │   (thread)    │   │ (test data)  │        │
│  └──────────────┘   └──────────────┘   └──────────────┘        │
└────────────────────────────────┬────────────────────────────────┘
                                 │ Raw bytes
                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│                     Serial Port (pyserial)                       │
│                    /dev/ttyUSB0 @ 921600                         │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📁 Cấu Trúc Files

```
protocol/
├── __init__.py         # Export public API
├── packet.py           # Packet definition + decode
├── serial_receiver.py  # Threaded serial reader
└── simulator.py        # Fake data generator for testing
```

---

## 📝 packet.py - Định Nghĩa Packet

### Mục Đích

Định nghĩa cấu trúc packet **giống hệt** với MCU side (`telemetry_packet.h`), đảm bảo PC có thể decode đúng data.

### Cấu Trúc Dữ Liệu

```python
@dataclass
class ImuData:
    """Data cho một IMU"""
    gyro_x: float = 0.0      # rad/s
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    accel_x: float = 0.0     # m/s²
    accel_y: float = 0.0
    accel_z: float = 0.0
    temperature: float = 0.0  # °C

@dataclass
class TelemetryData:
    """Full telemetry packet - 4 IMUs"""
    sequence: int = 0
    timestamp_us: int = 0
    
    # 4 Individual IMUs
    imu: List[ImuData] = field(...)
    
    # Magnetometer
    mag_x: float = 0.0   # Gauss
    mag_y: float = 0.0
    mag_z: float = 0.0
    
    # Barometer
    pressure: float = 0.0     # Pa
    baro_alt: float = 0.0     # m
    
    # GPS
    latitude: float = 0.0     # degrees
    longitude: float = 0.0
    gps_alt: float = 0.0      # m
    ground_speed: float = 0.0 # m/s
    heading: float = 0.0      # degrees
    fix_type: int = 0         # 0=none, 2=2D, 3=3D
    satellites: int = 0
    
    # Attitude (from EKF)
    quat_w: float = 1.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0
    roll: float = 0.0         # rad
    pitch: float = 0.0
    yaw: float = 0.0
    
    # Status
    health_level: int = 0     # 0=GOOD, 1=WARN, 2=CRIT, 3=FAIL
    healthy_imus: int = 0x0F  # Bitmask
    cpu_load: float = 0.0     # 0-100%
```

### Decode Function

```python
def decode_packet(data: bytes) -> Optional[TelemetryData]:
    """
    Decode 212-byte packet thành TelemetryData object.
    
    Args:
        data: Exactly 212 bytes
        
    Returns:
        TelemetryData nếu thành công
        None nếu CRC fail hoặc magic sai
    """
```

### CRC16-CCITT

```python
def crc16_ccitt(data: bytes) -> int:
    """
    Calculate CRC16-CCITT (0xFFFF init, 0x1021 poly)
    
    Phải giống implementation trên MCU!
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
        crc &= 0xFFFF
    return crc
```

---

## 📝 serial_receiver.py - Serial Reader Thread

### Mục Đích

Chạy trong **thread riêng** để đọc serial liên tục mà không block GUI. Khi có packet hợp lệ, emit Qt signal.

### Kiến Trúc

```
┌─────────────────────────────────────────────────────────────────┐
│                      SerialReceiver                              │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐                          ┌─────────────────┐   │
│  │ Worker      │   bytes   ┌──────────┐   │ Qt Signals      │   │
│  │ Thread      │──────────▶│ Packet   │──▶│                 │   │
│  │ (loop read) │           │ Decoder  │   │ packet_received │   │
│  └─────────────┘           └──────────┘   │ error_occurred  │   │
│                                           │ stats_updated   │   │
│                                           └─────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### State Machine Decoder

```python
class DecoderState(Enum):
    SYNC_1 = auto()   # Đợi byte 0xAA (little-endian first)
    SYNC_2 = auto()   # Đợi byte 0x55 (little-endian second)
    PAYLOAD = auto()  # Đọc 210 bytes còn lại

class PacketDecoder:
    """
    Xử lý byte-by-byte, handle noise và resync tự động.
    """
    def feed(self, data: bytes) -> list[TelemetryData]:
        """
        Feed raw bytes, trả về list packets đã decode.
        Thường trả về 0 hoặc 1 packet.
        """
```

### API

```python
class SerialReceiver(QObject):
    # Signals
    packet_received = Signal(object)   # TelemetryData
    connected = Signal()
    disconnected = Signal()
    error_occurred = Signal(str)
    stats_updated = Signal(object)     # ReceiverStats
    
    def connect(self, port: str, baudrate: int) -> bool:
        """Mở serial port và bắt đầu đọc"""
        
    def disconnect(self):
        """Đóng serial port"""
        
    @property
    def is_connected(self) -> bool:
        """Trạng thái kết nối"""
```

### Usage Example

```python
receiver = SerialReceiver()

# Connect signals
receiver.packet_received.connect(on_new_data)
receiver.error_occurred.connect(on_error)

# Start receiving
receiver.connect("/dev/ttyUSB0", 921600)

def on_new_data(data: TelemetryData):
    print(f"Gyro: {data.imu[0].gyro_x:.3f} rad/s")
```

---

## 📝 simulator.py - Test Data Generator

### Mục Đích

Generate dữ liệu giả lập để test GUI mà **không cần hardware UAV thật**. Rất hữu ích khi:
- Phát triển UI
- Demo cho người khác
- Test performance

### Đặc Điểm Simulation

- **Attitude:** Xoay từ từ (roll, pitch, yaw oscillation)
- **IMU:** 4 IMU với noise nhẹ, giá trị tương tự nhau
- **GPS:** Position cố định hoặc di chuyển chậm
- **Rate:** 100 Hz giống UAV thật

### API

```python
class TelemetrySimulator(QObject):
    # Signal giống SerialReceiver
    packet_received = Signal(object)
    
    def start(self):
        """Bắt đầu generate data @ 100 Hz"""
        
    def stop(self):
        """Dừng"""
        
    @property
    def is_running(self) -> bool:
        """Đang chạy?"""
```

### Usage

```python
# Trong demo mode, dùng simulator thay serial
if demo_mode:
    self._simulator.packet_received.connect(on_data)
    self._simulator.start()
else:
    self._receiver.packet_received.connect(on_data)
    self._receiver.connect(port, baud)
```

---

## 🔧 Synchronization Strategy

### Vấn Đề

Serial là byte stream liên tục. Làm sao biết packet bắt đầu từ đâu?

### Giải Pháp: Magic Numbers + State Machine

```
Incoming bytes:
... | noise | 0xAA | 0x55 | seq_lo | seq_hi | ... 208 more bytes ... | CRC | 0x55 | 0xAA | noise | 0xAA | ...
          │    │      │
          │    │      └── SYNC_2 → PAYLOAD (read 210 bytes)
          │    └── SYNC_1 → SYNC_2
          └── Reset

Packet:  [0xAA 0x55] [... 206 bytes ...] [CRC16] [0x55 0xAA]
           start                                    end
```

### Xử Lý Lỗi

```python
# CRC fail → discard packet, continue sync
if calculated_crc != packet_crc:
    self.crc_errors += 1
    self.reset()
    return None

# End magic wrong → possible corruption
if end_magic != 0xAA55:
    self.sync_errors += 1
    self.reset()
    return None
```

### Resync Sau Lỗi

```python
# Nếu đang đọc payload mà gặp magic mới → resync
if byte == 0xAA and len(buffer) > 10:
    # Có thể packet bị corrupt, thử sync lại
    self.reset()
    self.buffer.append(byte)
    self.state = SYNC_2
```

---

## 📊 Statistics

```python
@dataclass
class ReceiverStats:
    packets_received: int = 0
    crc_errors: int = 0
    sync_errors: int = 0
    sequence_drops: int = 0  # Missed packets
    bytes_received: int = 0
    
    @property
    def error_rate(self) -> float:
        total = self.packets_received + self.crc_errors
        return self.crc_errors / total if total > 0 else 0
```

---

## 🐛 Debug Tips

### 1. Dump Raw Bytes

```python
# Xem raw data trước khi decode
def on_raw(data: bytes):
    print(' '.join(f'{b:02x}' for b in data))
```

### 2. Check Sync

```bash
# Trong GCS, xem sync_errors
# Nếu cao → baudrate sai hoặc noise trên dây
```

### 3. Test Decode Offline

```bash
python3 test_decode.py captured_data.bin
```

---

## 🔗 Liên Kết

- **MCU packet definition:** `/apps/uav/telemetry_app/telemetry_packet.h`
- **Protocol architecture:** `/apps/uav/TELEMETRY_ARCHITECTURE.md`
- **GUI integration:** `/tools/uav_gcs/gui/`
