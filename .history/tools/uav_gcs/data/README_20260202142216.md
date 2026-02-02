# UAV GCS - Data Layer

## 📋 Tổng Quan

Module `data` quản lý **luồng dữ liệu** giữa protocol layer và GUI layer:

1. **Buffering:** Lưu trữ history trong ring buffer
2. **Routing:** Phân phối data tới các widgets
3. **Logging:** Ghi ra SQLite để phân tích offline
4. **Statistics:** Tính toán rate, latency, error count

### Vị Trí Trong Kiến Trúc

```
┌─────────────────────────────────────────────────────────────────┐
│                         GUI Layer                                │
│              (Updates via Qt Signals)                            │
└────────────────────────────────┬────────────────────────────────┘
                                 │ new_data signal
                                 │
┌────────────────────────────────┼────────────────────────────────┐
│                        DATA LAYER                                │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    DataManager                            │   │
│  │  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐     │   │
│  │  │ RingBuffer  │   │   Stats     │   │  Latest     │     │   │
│  │  │ (History)   │   │  Tracking   │   │   Data      │     │   │
│  │  └─────────────┘   └─────────────┘   └─────────────┘     │   │
│  └──────────────────────────────────────────────────────────┘   │
│                              │                                   │
│  ┌───────────────────────────┴──────────────────────────────┐   │
│  │                    DataLogger                             │   │
│  │          (Async SQLite Writer Thread)                     │   │
│  └──────────────────────────────────────────────────────────┘   │
└────────────────────────────────┬────────────────────────────────┘
                                 │
                                 │ TelemetryData objects
                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Protocol Layer                              │
│                   (SerialReceiver)                               │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📁 Cấu Trúc Files

```
data/
├── __init__.py         # Export public API
├── data_manager.py     # Central data coordinator
└── logger.py           # SQLite async logger
```

---

## 📝 data_manager.py - Trung Tâm Dữ Liệu

### Mục Đích

- **Nhận** packets từ SerialReceiver
- **Lưu** vào RingBuffer cho history
- **Emit** signals cho GUI widgets
- **Track** statistics (rate, errors)

### Class: RingBuffer

```python
class RingBuffer:
    """
    Thread-safe ring buffer cho telemetry history.
    
    Pattern: Multi-Producer Single-Consumer (MPSC)
    - Producer: SerialReceiver thread (push)
    - Consumer: GUI thread (get_last, get_all)
    """
    
    def __init__(self, max_size: int = 10000):
        self._buffer: deque[TelemetryData] = deque(maxlen=max_size)
        self._lock = threading.Lock()
    
    def push(self, data: TelemetryData):
        """Thread-safe push (gọi từ receiver thread)"""
        with self._lock:
            self._buffer.append(data)
    
    def get_last(self, n: int = 1) -> List[TelemetryData]:
        """Get last N items (gọi từ GUI thread)"""
        with self._lock:
            return list(self._buffer)[-n:]
    
    def get_range(self, start_us: int, end_us: int) -> List[TelemetryData]:
        """Get items trong khoảng thời gian"""
        with self._lock:
            return [d for d in self._buffer 
                    if start_us <= d.timestamp_us <= end_us]
```

### Class: DataManager

```python
class DataManager(QObject):
    """
    Central coordinator cho data flow.
    """
    
    # Signals cho GUI
    new_data = Signal(object)       # TelemetryData mới nhất
    stats_updated = Signal(object)  # DataStats mỗi giây
    
    def __init__(self, buffer_size: int = 10000):
        self.buffer = RingBuffer(max_size=buffer_size)
        self._latest_data: Optional[TelemetryData] = None
        self._stats = DataStats()
    
    def on_packet_received(self, data: TelemetryData):
        """Slot nhận data từ SerialReceiver"""
        # Lưu vào buffer
        self.buffer.push(data)
        
        # Update latest
        self._latest_data = data
        
        # Update stats
        self._stats.packets_received += 1
        
        # Emit cho GUI
        self.new_data.emit(data)
```

### Statistics

```python
@dataclass
class DataStats:
    packets_received: int = 0
    packets_dropped: int = 0      # Sequence gaps
    buffer_usage: float = 0.0     # 0.0 - 1.0
    update_rate_hz: float = 0.0   # Measured rate
    latency_ms: float = 0.0       # Estimated latency
```

### Rate Calculation

```python
def _calculate_rate(self):
    """Tính update rate thực tế"""
    now = time.time()
    elapsed = now - self._last_rate_time
    
    if elapsed >= 1.0:  # Mỗi giây
        count = self._stats.packets_received - self._last_count
        self._stats.update_rate_hz = count / elapsed
        
        self._last_rate_time = now
        self._last_count = self._stats.packets_received
        
        self.stats_updated.emit(self._stats)
```

---

## 📝 logger.py - Async SQLite Logger

### Mục Đích

Ghi telemetry ra **SQLite database** để phân tích sau. Chạy trong thread riêng để không block GUI.

### Kiến Trúc

```
┌─────────────────────────────────────────────────────────────────┐
│                      DataLogger                                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Main Thread                          Worker Thread              │
│  ┌─────────────┐                     ┌─────────────────────┐   │
│  │   log()     │──── Queue ────────▶│   _worker_loop()    │   │
│  │ (non-block) │                     │   while running:    │   │
│  └─────────────┘                     │     data = queue.get│   │
│                                      │     INSERT INTO...   │   │
│                                      └─────────────────────┘   │
│                                                  │               │
│                                                  ▼               │
│                                      ┌─────────────────────┐   │
│                                      │   SQLite File       │   │
│                                      │ telemetry_*.db      │   │
│                                      └─────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### Database Schema

```sql
-- Sessions table
CREATE TABLE sessions (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    start_time TEXT NOT NULL,
    end_time TEXT,
    description TEXT
);

-- Telemetry data table
CREATE TABLE telemetry (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id INTEGER NOT NULL,
    timestamp_us INTEGER NOT NULL,
    sequence INTEGER,
    
    -- IMU 0-3 (primary IMU for backward compat)
    gyro_x REAL, gyro_y REAL, gyro_z REAL,
    accel_x REAL, accel_y REAL, accel_z REAL,
    temperature REAL,
    
    -- Magnetometer
    mag_x REAL, mag_y REAL, mag_z REAL,
    
    -- Barometer
    pressure REAL,
    baro_alt REAL,
    
    -- GPS
    latitude REAL, longitude REAL, gps_alt REAL,
    ground_speed REAL, heading REAL,
    fix_type INTEGER, satellites INTEGER,
    
    -- Attitude
    quat_w REAL, quat_x REAL, quat_y REAL, quat_z REAL,
    roll REAL, pitch REAL, yaw REAL,
    
    -- Status
    health_level INTEGER,
    cpu_load REAL,
    
    FOREIGN KEY(session_id) REFERENCES sessions(id)
);

-- Index for time-based queries
CREATE INDEX idx_telemetry_time ON telemetry(timestamp_us);
```

### API

```python
class DataLogger:
    def start_session(self, description: str = "") -> str:
        """
        Bắt đầu session mới, tạo database file.
        
        Returns:
            Path tới database file
        """
        
    def stop_session(self):
        """Dừng recording, đóng database"""
        
    def log(self, data: TelemetryData):
        """
        Queue data để ghi (non-blocking).
        
        Nếu queue đầy, drop data và tăng counter.
        """
        
    @property
    def is_recording(self) -> bool:
        """Đang recording?"""
```

### Usage

```python
logger = DataLogger(log_dir="./logs")

# Start recording
db_path = logger.start_session("Flight test #1")
print(f"Logging to: {db_path}")

# Log data (từ receiver callback)
def on_data(data: TelemetryData):
    logger.log(data)

# Stop recording
logger.stop_session()
```

### Đọc Log

```python
import sqlite3
import pandas as pd

# Connect to database
conn = sqlite3.connect("logs/telemetry_20260202_120000.db")

# Read all telemetry
df = pd.read_sql("""
    SELECT timestamp_us, gyro_x, gyro_y, gyro_z, 
           roll, pitch, yaw
    FROM telemetry
    WHERE session_id = 1
    ORDER BY timestamp_us
""", conn)

# Plot
import matplotlib.pyplot as plt
plt.plot(df['timestamp_us'] / 1e6, df['roll'])
plt.xlabel('Time (s)')
plt.ylabel('Roll (rad)')
plt.show()
```

---

## 🔄 Data Flow Example

### Complete Flow

```
1. UART bytes arrive
        │
        ▼
2. SerialReceiver.packet_received.emit(data)
        │
        ├──▶ DataManager.on_packet_received()
        │           │
        │           ├──▶ RingBuffer.push(data)
        │           │
        │           ├──▶ Logger.log(data)
        │           │
        │           └──▶ new_data.emit(data)
        │                       │
        │           ┌───────────┼───────────┐
        │           ▼           ▼           ▼
        │    AttitudeView  SensorDisplay  StatusBar
        │    (30 FPS)      (50 Hz)        (1 Hz)
        │
        └──▶ Direct connections (nếu cần)
```

### Connection Setup

```python
# In FastMainWindow.__init__

# Receiver → DataManager
self._receiver.packet_received.connect(
    self._data_manager.on_packet_received
)

# DataManager → GUI widgets
self._data_manager.new_data.connect(
    self._on_new_data
)

# Stats update
self._data_manager.stats_updated.connect(
    self._update_status_bar
)
```

---

## 📊 Memory Management

### Buffer Sizing

```python
# 10000 samples @ 100 Hz = 100 seconds history
buffer_size = 10000

# Mỗi TelemetryData ~500 bytes
# Total: ~5 MB memory cho buffer
```

### Queue Overflow Handling

```python
def log(self, data: TelemetryData):
    if not self._running:
        return
    
    try:
        self._queue.put_nowait(data)
    except queue.Full:
        # Queue đầy, drop data
        self.queue_drops += 1
```

---

## 🐛 Debug Tips

### 1. Kiểm Tra Buffer Usage

```python
print(f"Buffer usage: {data_manager.buffer.usage * 100:.1f}%")
```

### 2. Xem Rate Thực Tế

```python
# Trong stats callback
def on_stats(stats: DataStats):
    print(f"Rate: {stats.update_rate_hz:.1f} Hz")
    print(f"Drops: {stats.packets_dropped}")
```

### 3. SQLite Query Performance

```python
# Dùng EXPLAIN để check query plan
cursor.execute("EXPLAIN QUERY PLAN SELECT * FROM telemetry WHERE timestamp_us > ?")
```

---

## 🔗 Liên Kết

- **Protocol layer:** `/tools/uav_gcs/protocol/`
- **GUI layer:** `/tools/uav_gcs/gui/`
- **SQLite docs:** https://www.sqlite.org/docs.html
