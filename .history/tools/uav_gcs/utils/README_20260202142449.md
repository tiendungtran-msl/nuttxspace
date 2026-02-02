# UAV GCS - Utils Module

## 📋 Tổng Quan

Module `utils` chứa các **utility functions** và **helper classes** được dùng chung trong toàn bộ GCS application.

---

## 📁 Files

```
utils/
├── __init__.py       # Export public API
└── quaternion.py     # Quaternion math utilities
```

---

## 📐 quaternion.py - Toán Học Quaternion

### Mục Đích

Cung cấp các hàm chuyển đổi giữa **quaternion** và **Euler angles** để hiển thị attitude.

### Convention

- **Quaternion:** Scalar-first format `(w, x, y, z)`
- **Euler:** ZYX convention (yaw → pitch → roll)
- **Angles:** Radians

### API

```python
from uav_gcs.utils.quaternion import quat_to_euler, euler_to_quat, quat_normalize

# Normalize quaternion
w, x, y, z = quat_normalize(w, x, y, z)

# Quaternion → Euler (radians)
roll, pitch, yaw = quat_to_euler(w, x, y, z)

# Euler → Quaternion
w, x, y, z = euler_to_quat(roll, pitch, yaw)
```

### Ví Dụ Sử Dụng

```python
import math
from uav_gcs.utils.quaternion import quat_to_euler

# Từ telemetry packet
qw, qx, qy, qz = data.quat_w, data.quat_x, data.quat_y, data.quat_z

# Chuyển sang Euler để hiển thị
roll, pitch, yaw = quat_to_euler(qw, qx, qy, qz)

# Hiển thị dạng degrees
print(f"Roll: {math.degrees(roll):.1f}°")
print(f"Pitch: {math.degrees(pitch):.1f}°")
print(f"Yaw: {math.degrees(yaw):.1f}°")
```

### Toán Học

**Quaternion to Euler (ZYX):**

```python
# Roll (X-axis rotation)
sinr_cosp = 2 * (w*x + y*z)
cosr_cosp = 1 - 2 * (x*x + y*y)
roll = atan2(sinr_cosp, cosr_cosp)

# Pitch (Y-axis rotation)
sinp = 2 * (w*y - z*x)
if |sinp| >= 1:
    pitch = copysign(π/2, sinp)  # Gimbal lock
else:
    pitch = asin(sinp)

# Yaw (Z-axis rotation)
siny_cosp = 2 * (w*z + x*y)
cosy_cosp = 1 - 2 * (y*y + z*z)
yaw = atan2(siny_cosp, cosy_cosp)
```

### Lưu Ý

1. **Gimbal Lock:** Khi pitch = ±90°, roll và yaw không xác định riêng biệt
2. **Normalization:** Luôn normalize quaternion trước khi convert
3. **Convention matching:** Đảm bảo MCU và PC dùng cùng convention

---

## 🔧 Extension

### Thêm Utility Mới

1. Tạo file mới trong `utils/`
2. Viết functions/classes
3. Export trong `__init__.py`

```python
# utils/my_util.py
def my_function():
    pass

# utils/__init__.py
from .my_util import my_function
```

---

## 🔗 Liên Kết

- **Sử dụng trong:** `/uav_gcs/gui/attitude_view.py`
- **MCU quaternion:** `/apps/uav/lib/mathlib/quaternion.hpp`
