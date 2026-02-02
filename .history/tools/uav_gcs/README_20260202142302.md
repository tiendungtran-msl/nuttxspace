# UAV Ground Control Station (uav_gcs)

## 📋 Tổng Quan

`uav_gcs` là **Python package** cung cấp giao diện đồ họa và công cụ để giám sát UAV từ máy tính. Package này được thiết kế theo kiến trúc **layered** với sự tách biệt rõ ràng giữa các tầng.

### Đặc Điểm Chính

| Feature | Mô Tả |
|---------|-------|
| **4 IMU Display** | Hiển thị dữ liệu từ 4 cảm biến riêng lẻ |
| **3D Visualization** | Mô hình UAV xoay theo quaternion thực |
| **100 Hz Update** | Nhận dữ liệu tốc độ cao từ MCU |
| **SQLite Logging** | Ghi log offline cho phân tích sau |
| **Demo Mode** | Test GUI không cần hardware |

---

## 📁 Cấu Trúc Package

```
uav_gcs/
├── __init__.py           # Package initialization
├── main.py               # Entry point (full version)
│
├── protocol/             # Tầng giao tiếp (thấp nhất)
│   ├── __init__.py
│   ├── packet.py         # Packet definition & decode
│   ├── serial_receiver.py# Threaded serial reader
│   └── simulator.py      # Fake data generator
│
├── data/                 # Tầng dữ liệu (giữa)
│   ├── __init__.py
│   ├── data_manager.py   # Central data coordinator
│   └── logger.py         # SQLite async writer
│
├── gui/                  # Tầng giao diện (cao nhất)
│   ├── __init__.py
│   ├── fast_main_window.py   # Optimized window
│   ├── main_window.py        # Full-featured window
│   ├── control_panel.py      # Left control panel
│   ├── sensor_display.py     # 4-IMU numeric display
│   ├── attitude_view.py      # 3D OpenGL view
│   ├── plot_panel.py         # Realtime plots
│   └── data_table.py         # Data table view
│
└── utils/                # Tiện ích
    ├── __init__.py
    └── quaternion.py     # Quaternion math
```

---

## 🏗️ Kiến Trúc Layers

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                                 GUI LAYER                                   │
│                         (PySide6 Qt Widgets)                                │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │FastMainWindow│ │ControlPanel│  │AttitudeView │  │SensorDisplay│        │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘        │
│         │                │                │                │                │
│         └────────────────┴────────────────┴────────────────┘                │
│                                   │                                         │
│                                   │ Qt Signals                              │
│                                   ▼                                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                DATA LAYER                                   │
│                      (Buffer, Stats, Logging)                               │
│                                                                             │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │                         DataManager                                 │    │
│  │  ┌───────────────┐   ┌───────────────┐   ┌───────────────┐         │    │
│  │  │  RingBuffer   │   │    Stats      │   │  Latest Data  │         │    │
│  │  │  (History)    │   │   Tracking    │   │   (Current)   │         │    │
│  │  └───────────────┘   └───────────────┘   └───────────────┘         │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│                                   │                                         │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │                         DataLogger                                  │    │
│  │          (Async SQLite Writer in separate thread)                   │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│                                   │                                         │
│                                   │ TelemetryData objects                   │
│                                   ▼                                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                             PROTOCOL LAYER                                  │
│                       (Serial Communication)                                │
│                                                                             │
│  ┌──────────────────┐   ┌──────────────────┐   ┌──────────────────┐        │
│  │  SerialReceiver  │   │   PacketDecoder  │   │   TelemetrySim   │        │
│  │  (Thread + Port) │   │  (State Machine) │   │   (Demo Mode)    │        │
│  └──────────────────┘   └──────────────────┘   └──────────────────┘        │
│                                   │                                         │
│                                   │ Raw bytes                               │
│                                   ▼                                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                            HARDWARE LAYER                                   │
│                     (pyserial + USB/UART)                                   │
│                                                                             │
│                    /dev/ttyUSB0 @ 921600 baud                               │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 🔄 Data Flow

### 1. Normal Mode (Serial)

```
UART bytes ──▶ SerialReceiver (thread)
                     │
                     ▼
              PacketDecoder (state machine)
                     │ Valid packet
                     ▼
              packet_received.emit(TelemetryData)
                     │
                     ├──▶ DataManager.on_packet()
                     │         │
                     │         ├──▶ RingBuffer.push()
                     │         ├──▶ Logger.log()
                     │         └──▶ new_data.emit()
                     │                   │
                     │         ┌─────────┴─────────┐
                     │         ▼                   ▼
                     │   AttitudeView         SensorDisplay
                     │   (30 FPS timer)       (50 Hz timer)
                     │
                     └──▶ Direct widget connections
```

### 2. Demo Mode (Simulator)

```
TelemetrySimulator (QTimer @ 100 Hz)
         │
         ▼ Generate fake data
packet_received.emit(TelemetryData)
         │
         └──▶ (Same flow as normal mode)
```

---

## 📦 Dependencies

```
PySide6>=6.4.0     # Qt6 bindings
pyserial>=3.5      # Serial port access
numpy>=1.21.0      # Numeric computing
vispy>=0.12.0      # 3D OpenGL rendering (optional)
```

### Optional

```
pyqtgraph>=0.13.0  # Realtime plotting (for full version)
```

---

## 🚀 Usage

### As Package

```python
from uav_gcs.protocol import SerialReceiver, TelemetryData
from uav_gcs.gui import FastMainWindow
from uav_gcs.data import DataManager

# Create components
receiver = SerialReceiver()
data_manager = DataManager()

# Connect
receiver.packet_received.connect(data_manager.on_packet_received)

# Start receiving
receiver.connect("/dev/ttyUSB0", 921600)
```

### As Application

```bash
# Fast mode (recommended)
python3 -m uav_gcs.main

# Or via entry script
python3 run_gcs_fast.py
```

---

## 🎨 Theme

Dark theme được áp dụng globally:

```python
palette = QPalette()
palette.setColor(QPalette.Window, QColor(25, 25, 25))
palette.setColor(QPalette.WindowText, QColor(200, 200, 200))
palette.setColor(QPalette.Base, QColor(30, 30, 30))
# ...
app.setPalette(palette)
```

---

## 📊 Performance Considerations

### 1. Throttled Updates

```python
# GUI updates không cần 100 Hz
self._attitude_timer.setInterval(33)   # 30 FPS
self._sensor_timer.setInterval(20)     # 50 Hz
self._stats_timer.setInterval(1000)    # 1 Hz
```

### 2. Value Change Detection

```python
# Chỉ update label khi giá trị thay đổi đáng kể
if abs(new_value - last_value) < threshold:
    return  # Skip update
```

### 3. Async Logging

```python
# Logger chạy trong thread riêng
# Main thread không bị block bởi disk I/O
```

---

## 🐛 Troubleshooting

### VisPy 3D View Không Hoạt Động

```bash
# Cài OpenGL
sudo apt install libgl1-mesa-glx

# GCS sẽ tự fallback sang text mode nếu VisPy fail
```

### Serial Permission Denied

```bash
sudo usermod -a -G dialout $USER
# Logout và login lại
```

### High CPU Usage

1. Giảm update rate của timers
2. Tắt plot panel nếu không cần
3. Dùng `fast_main_window` thay `main_window`

---

## 🔗 Liên Kết

- **MCU Telemetry:** `/apps/uav/telemetry_app/`
- **Protocol Docs:** `/apps/uav/TELEMETRY_ARCHITECTURE.md`
- **Tools README:** `/tools/README.md`
