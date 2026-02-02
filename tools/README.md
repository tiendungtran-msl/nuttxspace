# 🛠️ UAV Ground Control Tools

## 📋 Tổng Quan

Thư mục này chứa các công cụ hỗ trợ phát triển và giám sát UAV, được viết bằng Python. Mục đích chính:

1. **Nhận dữ liệu telemetry** từ UAV qua UART/USB
2. **Hiển thị realtime** các thông số cảm biến và trạng thái
3. **Visualize** tư thế UAV trong không gian 3D
4. **Ghi log** để phân tích offline

### Sơ Đồ Tổng Quan

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              UAV (STM32H743)                                │
│  telemetry_app → UART TX → 212-byte packets @ 100 Hz                       │
└───────────────────────────────┬─────────────────────────────────────────────┘
                                │ USB Cable
                                ▼
┌───────────────────────────────────────────────────────────────────────────────┐
│                           PC (Python Tools)                                   │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐           │
│  │  Serial Thread  │───▶│  Packet Decoder │───▶│  Data Manager   │           │
│  │  (Non-blocking) │    │  + CRC Check    │    │  (Ring Buffer)  │           │
│  └─────────────────┘    └─────────────────┘    └────────┬────────┘           │
│                                                         │                     │
│              ┌──────────────────────────────────────────┼──────────┐         │
│              │                                          │          │         │
│              ▼                                          ▼          ▼         │
│  ┌─────────────────────┐    ┌─────────────────────┐    ┌────────────────┐   │
│  │  3D Attitude View   │    │  Sensor Display     │    │  SQLite Logger │   │
│  │  (VisPy/OpenGL)     │    │  (Fast Text Update) │    │  (Async Write) │   │
│  └─────────────────────┘    └─────────────────────┘    └────────────────┘   │
└───────────────────────────────────────────────────────────────────────────────┘
```

---

## 📁 Cấu Trúc Thư Mục

```
tools/
├── README.md                 # File này
├── requirements.txt          # Python dependencies
├── run_gcs_fast.py           # Entry point chính - chạy GCS nhanh
├── test_decode.py            # Test packet decode offline
├── dump_raw.py               # Dump raw serial data để debug
│
├── uav_gcs/                  # Package Python chính
│   ├── __init__.py
│   ├── main.py               # Entry point thay thế
│   │
│   ├── protocol/             # Layer giao tiếp với UAV
│   │   ├── packet.py         # Định nghĩa packet structure
│   │   ├── serial_receiver.py# Thread nhận serial
│   │   └── simulator.py      # Giả lập data để test GUI
│   │
│   ├── gui/                  # Giao diện đồ họa (PySide6)
│   │   ├── fast_main_window.py   # Cửa sổ chính tối ưu
│   │   ├── main_window.py        # Cửa sổ chính đầy đủ
│   │   ├── control_panel.py      # Panel điều khiển trái
│   │   ├── sensor_display.py     # Hiển thị sensor số
│   │   ├── attitude_view.py      # 3D visualization
│   │   ├── plot_panel.py         # Biểu đồ realtime
│   │   └── data_table.py         # Bảng dữ liệu
│   │
│   ├── data/                 # Quản lý dữ liệu
│   │   ├── data_manager.py   # Ring buffer + signal routing
│   │   └── logger.py         # SQLite async logger
│   │
│   └── utils/                # Tiện ích
│       └── quaternion.py     # Quaternion math
│
└── logs/                     # Thư mục chứa log files
    └── telemetry_*.db        # SQLite database files
```

---

## 🚀 Cài Đặt và Chạy

### 1. Cài Đặt Dependencies

```bash
cd /home/msi-leibniz/nuttxspace/tools
pip install -r requirements.txt
```

**Dependencies chính:**
- `PySide6` - Qt6 bindings cho GUI
- `pyserial` - Đọc serial port
- `vispy` - 3D visualization (OpenGL)
- `numpy` - Tính toán số

### 2. Chạy Ground Control Station

```bash
# Chế độ nhanh (khuyến nghị) - không có plot nặng
python3 run_gcs_fast.py

# Hoặc chế độ đầy đủ với plots
python3 -m uav_gcs.main
```

### 3. Kết Nối UAV

1. Cắm USB từ UAV vào PC
2. Trong GCS, chọn port (thường `/dev/ttyUSB0`)
3. Chọn baudrate `921600`
4. Click **Connect**

### 4. Chế Độ Demo (Không Cần UAV)

Click **Demo Mode** để GCS tự generate dữ liệu giả lập, giúp test GUI mà không cần hardware thật.

---

## 📦 Packet Protocol

### Cấu Trúc Packet (212 bytes)

```
┌──────────────────────────────────────────────────────────────────────────┐
│                     TELEMETRY PACKET (212 bytes)                         │
├─────────┬────────┬───────────────────────────────────────────────────────┤
│ Offset  │  Size  │                     Field                             │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│ HEADER (8 bytes)                                                          │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│   0     │   2    │ Magic start (0x55AA)                                  │
│   2     │   2    │ Sequence number                                        │
│   4     │   4    │ Timestamp (µs since boot)                              │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│ IMU DATA (28 bytes × 4 = 112 bytes)                                       │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│   8     │   28   │ IMU 0: Gx,Gy,Gz,Ax,Ay,Az,Temp (7 floats)              │
│  36     │   28   │ IMU 1: Gx,Gy,Gz,Ax,Ay,Az,Temp (7 floats)              │
│  64     │   28   │ IMU 2: Gx,Gy,Gz,Ax,Ay,Az,Temp (7 floats)              │
│  92     │   28   │ IMU 3: Gx,Gy,Gz,Ax,Ay,Az,Temp (7 floats)              │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│ SENSOR DATA                                                               │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│ 120     │   12   │ Magnetometer: Mx,My,Mz (3 floats)                     │
│ 132     │    8   │ Barometer: Pressure,Altitude (2 floats)               │
│ 140     │   24   │ GPS: lat,lon,alt,speed,heading,fix,sats,hdop,vdop     │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│ ATTITUDE (32 bytes)                                                       │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│ 164     │   16   │ Quaternion: W,X,Y,Z (4 floats)                        │
│ 180     │   12   │ Euler: Roll,Pitch,Yaw (3 floats)                      │
│ 192     │    4   │ EKF innovation variance                               │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│ STATUS (12 bytes)                                                         │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│ 196     │    1   │ Health level (0-3)                                     │
│ 197     │    1   │ Healthy IMU bitmask                                    │
│ 198     │    1   │ Sensor status flags                                    │
│ 199     │    1   │ Reserved                                               │
│ 200     │    2   │ CPU load (0-1000)                                      │
│ 202     │    2   │ Battery voltage (mV)                                   │
│ 204     │    4   │ Loop count                                             │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│ FOOTER (4 bytes)                                                          │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│ 208     │    2   │ CRC16-CCITT                                            │
│ 210     │    2   │ Magic end (0xAA55)                                     │
└─────────┴────────┴───────────────────────────────────────────────────────┘
```

### Sync Strategy

```python
State Machine:
┌─────────┐  0xAA   ┌─────────┐  0x55   ┌─────────┐
│ SYNC_1  │────────▶│ SYNC_2  │────────▶│ PAYLOAD │
│ (Idle)  │◀────────│         │◀────────│         │
└─────────┘  other  └─────────┘  other  └─────────┘
                                            │
                                 212 bytes  │
                                            ▼
                                    ┌───────────────┐
                                    │ Decode + CRC  │
                                    │    Check      │
                                    └───────────────┘
```

---

## 🖥️ Giao Diện GUI

### Layout Chính

```
┌─────────────────────────────────────────────────────────────────────┐
│ UAV GCS - Fast Debug View                                    [_][□][X]│
├─────────────┬───────────────────────────────────────────────────────┤
│             │                                                        │
│  Connection │              3D Attitude Viewer                        │
│  ┌────────┐ │              (OpenGL + VisPy)                         │
│  │ Port ▼ │ │                                                        │
│  │ Baud ▼ │ │                  ┌─────────┐                          │
│  │Connect │ │                  │   UAV   │ ← Rotates with attitude  │
│  └────────┘ │                  │  Model  │                          │
│             │                  └─────────┘                          │
│  Demo Mode  │                                                        │
│  ┌────────┐ │          Roll: 5.2°  Pitch: -2.1°  Yaw: 45.0°        │
│  │ Start  │ ├───────────────────────────────────────────────────────┤
│  └────────┘ │                                                        │
│             │     IMU Fusion Panel (4 sensors)                       │
│  Status     │  ┌─────┬──────┬──────┬──────┬──────┬──────┬──────┐   │
│  ┌────────┐ │  │ ☑  │ IMU0 │ 0.001│-0.002│ 0.003│  0.1 │  9.8 │   │
│  │ Rate   │ │  │ ☑  │ IMU1 │ 0.001│-0.001│ 0.002│  0.1 │  9.8 │   │
│  │ Packets│ │  │ ☑  │ IMU2 │ 0.002│-0.002│ 0.003│  0.2 │  9.7 │   │
│  │ Errors │ │  │ ☑  │ IMU3 │ 0.001│-0.003│ 0.002│  0.1 │  9.8 │   │
│  └────────┘ │  └─────┴──────┴──────┴──────┴──────┴──────┴──────┘   │
│             │                                                        │
├─────────────┴───────────────────────────────────────────────────────┤
│ Connected: /dev/ttyUSB0 @ 921600        Rate: 100.0 Hz              │
└─────────────────────────────────────────────────────────────────────┘
```

### Các Thành Phần

| Component | Mô Tả | Update Rate |
|-----------|-------|-------------|
| Control Panel | Kết nối, demo mode, recording | User action |
| 3D Attitude | Hiển thị UAV quay theo quaternion | 30 FPS |
| Sensor Display | Số liệu 4 IMU dạng bảng | 50 Hz |
| Status Bar | Connection status, rate | 1 Hz |

---

## 📊 Data Logging

### Format SQLite

```sql
-- Bảng sessions
CREATE TABLE sessions (
    id INTEGER PRIMARY KEY,
    start_time TEXT,
    end_time TEXT,
    description TEXT
);

-- Bảng telemetry
CREATE TABLE telemetry (
    id INTEGER PRIMARY KEY,
    session_id INTEGER,
    timestamp_us INTEGER,
    sequence INTEGER,
    gyro_x REAL, gyro_y REAL, gyro_z REAL,
    accel_x REAL, accel_y REAL, accel_z REAL,
    roll REAL, pitch REAL, yaw REAL,
    -- ... more fields
    FOREIGN KEY(session_id) REFERENCES sessions(id)
);
```

### Đọc Log

```python
import sqlite3
import pandas as pd

# Đọc toàn bộ telemetry
conn = sqlite3.connect("logs/telemetry_20260202_120000.db")
df = pd.read_sql("SELECT * FROM telemetry", conn)
print(df.head())
```

---

## 🐛 Troubleshooting

### 1. Không Thấy Serial Port

```bash
# Kiểm tra device
ls /dev/ttyUSB*

# Thêm user vào group dialout
sudo usermod -a -G dialout $USER
# Logout và login lại
```

### 2. VisPy Không Hoạt Động

```bash
# Cài OpenGL
sudo apt install libgl1-mesa-glx

# Nếu vẫn lỗi, GCS sẽ fallback sang text-only mode
```

### 3. Packet Không Sync

- Kiểm tra baudrate phải khớp (921600)
- Kiểm tra cable UART
- Xem `sync_errors` trong status

### 4. Rate Thấp Hơn 100 Hz

- USB latency: `cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`
- Giảm latency: `echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`

---

## 🔗 Liên Kết

- **Protocol MCU side:** `/apps/uav/telemetry_app/`
- **Packet definition:** `/apps/uav/telemetry_app/telemetry_packet.h`
- **Architecture doc:** `/apps/uav/TELEMETRY_ARCHITECTURE.md`
