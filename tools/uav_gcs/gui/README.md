# UAV GCS - GUI Layer

## 📋 Tổng Quan

Module `gui` chứa tất cả các widget và windows cho giao diện người dùng, được xây dựng trên **PySide6** (Qt6 for Python).

### Triết Lý Thiết Kế

1. **Performance First:** Tối ưu cho hiển thị 100 Hz data
2. **Modularity:** Mỗi widget độc lập, có thể reuse
3. **Dark Theme:** Dễ nhìn khi debug lâu
4. **Responsive:** Không freeze khi nhận data liên tục

---

## 📁 Cấu Trúc Files

```
gui/
├── __init__.py           # Export public widgets
├── fast_main_window.py   # Cửa sổ chính (tối ưu, không plot)
├── main_window.py        # Cửa sổ chính (đầy đủ với plots)
├── control_panel.py      # Panel điều khiển bên trái
├── sensor_display.py     # Hiển thị 4 IMU dạng số
├── attitude_view.py      # 3D OpenGL visualization
├── plot_panel.py         # Realtime plots (PyQtGraph)
└── data_table.py         # Bảng dữ liệu chi tiết
```

---

## 🖼️ fast_main_window.py - Cửa Sổ Chính Tối Ưu

### Mục Đích

Cửa sổ chính **tối ưu cho tốc độ**, không có các plot nặng. Thích hợp cho:
- Debug realtime với data rate cao
- Máy tính cấu hình thấp
- Khi chỉ cần xem số liệu nhanh

### Layout

```
┌─────────────────────────────────────────────────────────────────────┐
│ Menu Bar                                                            │
├────────────┬────────────────────────────────────────────────────────┤
│            │                                                        │
│  Control   │              3D Attitude Viewer                        │
│   Panel    │              (VisPy OpenGL)                           │
│   220px    │                                                        │
│            │                                                        │
│            ├────────────────────────────────────────────────────────┤
│            │                                                        │
│            │              Fast Sensor Display                       │
│            │              (Text-based, 4 IMUs)                     │
│            │                                                        │
├────────────┴────────────────────────────────────────────────────────┤
│ Status Bar                                      Rate: 100.0 Hz      │
└─────────────────────────────────────────────────────────────────────┘
```

### Components

```python
class FastMainWindow(QMainWindow):
    def __init__(self):
        # Core components
        self._receiver = SerialReceiver()      # Nhận serial
        self._simulator = TelemetrySimulator() # Demo mode
        self._data_manager = DataManager()     # Buffer data
        self._logger = DataLogger()            # Ghi log
        
        # UI components
        self._control_panel = ControlPanel()   # Bên trái
        self._attitude_view = AttitudeViewer() # 3D view
        self._sensor_display = FastSensorDisplay()  # Số liệu
```

### Update Flow

```
SerialReceiver.packet_received
        │
        ▼
DataManager.on_packet()
        │
        ├──▶ RingBuffer.push()
        │
        └──▶ new_data.emit(packet)
                    │
        ┌───────────┴───────────────┐
        ▼                           ▼
AttitudeViewer.update()    SensorDisplay.update()
   (30 FPS timer)              (50 Hz timer)
```

### Timer Strategy

Để tránh update quá nhanh gây lag:

```python
# GUI update timers (throttled)
self._attitude_timer = QTimer()
self._attitude_timer.setInterval(33)   # ~30 FPS
self._attitude_timer.timeout.connect(self._update_attitude)

self._sensor_timer = QTimer()
self._sensor_timer.setInterval(20)     # 50 Hz
self._sensor_timer.timeout.connect(self._update_sensors)
```

---

## 🎛️ control_panel.py - Panel Điều Khiển

### Mục Đích

Chứa tất cả controls cho kết nối và recording ở bên trái màn hình.

### Components

```
┌─────────────────────┐
│     Connection      │
│ ┌─────────────────┐ │
│ │ Port: [____▼]   │ │
│ │ Baud: [921600▼] │ │
│ │ [Connect]       │ │
│ └─────────────────┘ │
├─────────────────────┤
│    Demo Mode        │
│ [Start Demo]        │
├─────────────────────┤
│     Recording       │
│ [● Record]          │
│ [Reset Data]        │
├─────────────────────┤
│      Status         │
│ ● Connected         │
│ Rate: 100.0 Hz      │
│ Packets: 12345      │
│ Errors: 0           │
└─────────────────────┘
```

### Signals

```python
class ControlPanel(QWidget):
    # Signals to main window
    connect_requested = Signal(str, int)    # port, baudrate
    disconnect_requested = Signal()
    record_requested = Signal(bool)         # start/stop
    demo_requested = Signal(bool)           # start/stop
    reset_data_requested = Signal()
```

### Port Detection

```python
def _refresh_ports(self):
    """Auto-detect serial ports"""
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    
    self._port_combo.clear()
    for port in ports:
        # Show friendly name
        self._port_combo.addItem(
            f"{port.device} - {port.description}",
            port.device
        )
```

---

## 📊 sensor_display.py - Hiển Thị Sensor

### Mục Đích

Hiển thị dữ liệu 4 IMU dạng bảng số, **tối ưu cho update nhanh**.

### Layout

```
┌──────────────────────────────────────────────────────────────────────┐
│ IMU Fusion Panel (4 sensors)                                         │
├─────┬──────┬────────┬────────┬────────┬────────┬────────┬────────┬──┤
│  ☑  │ IMU  │   Gx   │   Gy   │   Gz   │   Ax   │   Ay   │   Az   │T │
├─────┼──────┼────────┼────────┼────────┼────────┼────────┼────────┼──┤
│ [✓] │  0   │ +0.001 │ -0.002 │ +0.003 │ +0.123 │ -0.045 │ +9.812 │25│
│ [✓] │  1   │ +0.002 │ -0.001 │ +0.002 │ +0.118 │ -0.050 │ +9.805 │25│
│ [✓] │  2   │ +0.001 │ -0.003 │ +0.004 │ +0.125 │ -0.042 │ +9.798 │26│
│ [✓] │  3   │ +0.000 │ -0.002 │ +0.003 │ +0.120 │ -0.048 │ +9.810 │25│
└─────┴──────┴────────┴────────┴────────┴────────┴────────┴────────┴──┘
```

### Tối Ưu Hóa

```python
class ValueLabel(QLabel):
    """High-performance value label"""
    
    def set_value(self, value: float, force: bool = False):
        # Chỉ update nếu thay đổi đáng kể
        if not force and self._last_value is not None:
            if abs(value - self._last_value) < self._threshold:
                return  # Skip update
        
        self._last_value = value
        self.setText(f"{value:>8.3f}")
```

### Color Coding

```python
# Gyro: Green khi nhỏ, Yellow khi xoay, Red khi quá nhanh
if abs(gyro) < 0.1:
    label.set_color('green')
elif abs(gyro) < 1.0:
    label.set_color('yellow')
else:
    label.set_color('red')
```

### IMU Selection

Checkbox cho phép chọn IMU nào tham gia fusion (để debug):

```python
class ImuFusionPanel(QGroupBox):
    imu_selection_changed = Signal(list)  # [0, 1, 2, 3]
    
    def _on_checkbox_changed(self):
        selected = [i for i, cb in enumerate(self._checkboxes) if cb.isChecked()]
        self.imu_selection_changed.emit(selected)
```

---

## 🎨 attitude_view.py - 3D Visualization

### Mục Đích

Hiển thị UAV trong không gian 3D, xoay theo quaternion từ EKF.

### Technology Stack

- **VisPy**: OpenGL wrapper cho Python
- **SceneCanvas**: Container cho 3D scene
- **TurntableCamera**: User có thể xoay góc nhìn

### 3D Scene Setup

```python
def _setup_3d_scene(self):
    # Create canvas
    self._canvas = scene.SceneCanvas(
        bgcolor='#1e1e1e',  # Dark background
        keys='interactive'  # Mouse controls
    )
    
    # Camera
    self._view.camera = scene.TurntableCamera(
        elevation=30,   # Nhìn từ trên xuống 30°
        azimuth=45,     # Xoay 45° quanh trục Z
        distance=5,     # Khoảng cách
        fov=60          # Field of view
    )
    
    # Grid reference
    grid = scene.visuals.GridLines()
    self._view.add(grid)
    
    # UAV model (simplified box)
    self._add_uav_model()
    
    # Axes (X=red, Y=green, Z=blue)
    self._add_axes()
```

### Quaternion to Rotation

```python
def update_attitude(self, qw: float, qx: float, qy: float, qz: float):
    """
    Update UAV orientation từ quaternion.
    Convention: w, x, y, z (scalar first)
    """
    # Convert to 4x4 rotation matrix
    rot = quaternion_to_rotation_matrix(qw, qx, qy, qz)
    
    # Apply to UAV model
    self._uav_transform.matrix = rot
```

### Euler Display

```python
# Hiển thị Roll/Pitch/Yaw cho dễ đọc
roll, pitch, yaw = quat_to_euler(qw, qx, qy, qz)
self._roll_label.setText(f"Roll: {math.degrees(roll):+.1f}°")
self._pitch_label.setText(f"Pitch: {math.degrees(pitch):+.1f}°")
self._yaw_label.setText(f"Yaw: {math.degrees(yaw):+.1f}°")
```

### Fallback Mode

Nếu VisPy không cài được (OpenGL issues):

```python
if not VISPY_AVAILABLE:
    # Simple text display
    self._fallback_label = QLabel("3D View unavailable\n(VisPy not installed)")
```

---

## 📈 plot_panel.py - Realtime Plots

### Mục Đích

Biểu đồ realtime cho phân tích signal. **Chỉ có trong main_window.py (full version)**.

### Plots Có Sẵn

| Plot | Data | Y-axis |
|------|------|--------|
| Gyroscope | Gx, Gy, Gz | rad/s |
| Accelerometer | Ax, Ay, Az | m/s² |
| Attitude | Roll, Pitch, Yaw | degrees |
| Altitude | Baro altitude | meters |

### Performance Considerations

PyQtGraph có thể nặng với data rate cao:

```python
# Chỉ plot 500 samples gần nhất
MAX_PLOT_POINTS = 500

# Downsample nếu cần
if len(data) > MAX_PLOT_POINTS:
    step = len(data) // MAX_PLOT_POINTS
    data = data[::step]

# Update 30 FPS thay vì 100 Hz
self._plot_timer.setInterval(33)
```

---

## 🎨 Styling

### Dark Theme

```python
# Applied in main entry point
palette = QPalette()
palette.setColor(QPalette.Window, QColor(25, 25, 25))
palette.setColor(QPalette.WindowText, QColor(200, 200, 200))
palette.setColor(QPalette.Base, QColor(30, 30, 30))
# ...
app.setPalette(palette)
```

### Monospace Font for Numbers

```python
font = QFont("Consolas, Monaco, monospace", 10)
font.setStyleHint(QFont.Monospace)
label.setFont(font)
```

### LED-style Status Indicators

```python
class StatusIndicator(QFrame):
    COLORS = {
        'off': '#404040',
        'green': '#00ff00',
        'yellow': '#ffff00',
        'red': '#ff0000'
    }
    
    def set_color(self, color: str):
        self.setStyleSheet(f"""
            background-color: {self.COLORS[color]};
            border-radius: 6px;
        """)
```

---

## 🔧 Extension Points

### Thêm Widget Mới

1. Tạo file mới trong `gui/`
2. Inherit từ `QWidget`
3. Export trong `__init__.py`
4. Thêm vào main window layout

```python
# my_widget.py
class MyWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()
    
    def update_data(self, data: TelemetryData):
        # Called when new data arrives
        pass
```

### Custom Color Scheme

```python
# Modify palette colors in run_gcs_fast.py
palette.setColor(QPalette.Highlight, QColor(0, 100, 180))
```

---

## 🐛 Debug Tips

### 1. Widget Không Update

```python
# Kiểm tra signal connections
print(f"Receivers: {signal.receivers()}")
```

### 2. GUI Freeze

```python
# Đảm bảo không block main thread
# Dùng QTimer thay vì while loop
```

### 3. High CPU Usage

```python
# Giảm update rate
self._timer.setInterval(50)  # 20 Hz thay vì 100 Hz
```

---

## 🔗 Liên Kết

- **Data flow:** `/tools/uav_gcs/data/`
- **Protocol:** `/tools/uav_gcs/protocol/`
- **Qt documentation:** https://doc.qt.io/qtforpython-6/
