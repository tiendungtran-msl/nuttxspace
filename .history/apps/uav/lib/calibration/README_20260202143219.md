# lib/calibration - Sensor Calibration

## 📋 Tổng Quan

Module `calibration` cung cấp **thư viện hiệu chuẩn sensor** cho IMU:
- **Bias correction:** Loại bỏ offset tĩnh
- **Scale correction:** Hiệu chỉnh độ nhạy các trục
- **Rotation correction:** Xoay từ sensor frame sang body frame

---

## 📁 Files

```
calibration/
├── Makefile
├── sensor_calibration.hpp    # API và định nghĩa
└── sensor_calibration.cpp    # Implementation
```

---

## 🎯 Tại Sao Cần Calibration?

### Raw Sensor = Nhiều Lỗi

```
Lỗi Accelerometer:
├── Bias (offset): Đọc khác 0 khi không có gia tốc
├── Scale: Độ nhạy khác nhau trên X, Y, Z
├── Misalignment: Trục sensor lệch với body frame
└── Nonlinearity: Không tuyến tính (advanced)

Ví dụ: Để yên trên bàn, accelerometer đo:
  Raw:  [0.05, -0.12, 9.65] m/s²
  True: [0.00,  0.00, 9.81] m/s²
        ↑       ↑      ↑
       bias   bias   scale error
```

### Calibrated = Chính Xác

```
                 ┌─────────────────────────────────────────┐
                 │          CALIBRATION MODEL              │
Raw data ───────▶│                                         │──────▶ Corrected
[x,y,z]          │  corrected = R × ((raw - bias) ⊙ scale) │        [x,y,z]
                 │                                         │
                 │  R = rotation matrix (3×3)              │
                 │  bias = offset vector [3]               │
                 │  scale = scale factor vector [3]        │
                 └─────────────────────────────────────────┘
```

---

## 📐 Calibration Model

### Phương Trình

$$
\vec{v}_{corrected} = \mathbf{R} \cdot \left( (\vec{v}_{raw} - \vec{b}) \odot \vec{s} \right)
$$

Trong đó:
- $\vec{v}_{raw}$: Raw sensor reading
- $\vec{b}$: Bias (offset) vector
- $\vec{s}$: Scale factor vector
- $\odot$: Element-wise multiplication
- $\mathbf{R}$: Rotation matrix (sensor → body frame)
- $\vec{v}_{corrected}$: Calibrated output

### Ví Dụ Số

```
Raw accel:  [0.15, -0.08, 9.50] m/s²
Bias:       [0.10,  0.05, 0.00] m/s²
Scale:      [1.00,  0.98, 1.03]
Rotation:   Identity (không xoay)

Step 1: Subtract bias
  [0.15 - 0.10, -0.08 - 0.05, 9.50 - 0.00]
= [0.05, -0.13, 9.50]

Step 2: Apply scale
  [0.05 × 1.00, -0.13 × 0.98, 9.50 × 1.03]
= [0.05, -0.127, 9.785]

Step 3: Apply rotation (identity = no change)
= [0.05, -0.127, 9.785]

Corrected: [0.05, -0.127, 9.785] m/s² ≈ [0, 0, 9.81]
```

---

## 🔧 API

### Accelerometer Calibration

```cpp
#include <uav/lib/calibration/sensor_calibration.hpp>

using namespace calibration;

Accelerometer accel_cal;

// Set bias (offset)
Vector3f bias(0.10f, 0.05f, 0.00f);  // m/s²
accel_cal.set_offset(bias);

// Set scale factors
Vector3f scale(1.00f, 0.98f, 1.03f);
accel_cal.set_scale(scale);

// Set rotation (sensor → body)
Dcmf rotation;  // Identity by default
// Hoặc tạo từ Euler angles nếu cần
accel_cal.set_rotation(rotation);

// Apply correction
Vector3f raw(ax, ay, az);
Vector3f corrected = accel_cal.correct(raw);
```

### Gyroscope Calibration

```cpp
Gyroscope gyro_cal;

// Gyro bias (drift)
Vector3f bias(0.001f, -0.002f, 0.0005f);  // rad/s
gyro_cal.set_offset(bias);

// Scale thường rất gần 1.0
Vector3f scale(1.0f, 1.0f, 1.0f);
gyro_cal.set_scale(scale);

// Apply
Vector3f raw_gyro(gx, gy, gz);
Vector3f corrected = gyro_cal.correct(raw_gyro);
```

---

## 📊 Data Structures

### Vector3f

```cpp
struct Vector3f {
    float x, y, z;
    
    Vector3f();
    Vector3f(float x_, float y_, float z_);
    
    float norm() const;           // Độ lớn
    void zero();                  // Set to [0,0,0]
    bool is_finite() const;       // Check NaN/Inf
    
    // Operators
    Vector3f operator+(const Vector3f&) const;
    Vector3f operator-(const Vector3f&) const;
    Vector3f operator*(float) const;
    Vector3f emult(const Vector3f&) const;    // Element-wise ×
    Vector3f edivide(const Vector3f&) const;  // Element-wise ÷
};
```

### Dcmf (Direction Cosine Matrix)

```cpp
class Dcmf {
public:
    float data[3][3];
    
    Dcmf();                           // Identity matrix
    Vector3f operator*(const Vector3f&) const;  // Matrix × vector
    Dcmf T() const;                   // Transpose
    static Dcmf identity();
};
```

---

## 🔄 Calibration Procedure

### 1. Gyro Bias Calibration

**Phương pháp:** Để yên UAV, đo trung bình 1000+ samples

```cpp
// Calibration routine
void calibrate_gyro(Gyroscope& cal)
{
    Vector3f sum(0, 0, 0);
    const int N = 1000;
    
    for (int i = 0; i < N; i++) {
        Vector3f raw = read_gyro_raw();
        sum = sum + raw;
        usleep(1000);  // 1 kHz
    }
    
    Vector3f bias = sum * (1.0f / N);
    cal.set_offset(bias);
}
```

### 2. Accelerometer Bias/Scale Calibration

**Phương pháp: 6-Point Calibration**

```
Đặt UAV ở 6 hướng, đo gravity vector:
  +X up, -X up, +Y up, -Y up, +Z up, -Z up

Với mỗi vị trí:
  Expected: [0, 0, ±9.81] m/s² (tuỳ hướng)
  Measured: [ax, ay, az]
```

```cpp
// Simplified 2-point per axis
void calibrate_accel_axis(int axis, float& bias, float& scale)
{
    // Measure with +gravity on axis
    float plus_g = measure_avg(axis, AXIS_UP);
    
    // Measure with -gravity on axis
    float minus_g = measure_avg(axis, AXIS_DOWN);
    
    // Expected: +9.81 và -9.81
    bias = (plus_g + minus_g) / 2.0f;
    scale = 2.0f * GRAVITY_MSS / (plus_g - minus_g);
}
```

### 3. Rotation Calibration

**Khi nào cần:** Sensor không gắn thẳng với body frame

```cpp
// Ví dụ: sensor xoay 90° quanh Z
// sensor X = body Y
// sensor Y = -body X
Dcmf R;
R.data[0][0] = 0;  R.data[0][1] = 1;  R.data[0][2] = 0;
R.data[1][0] = -1; R.data[1][1] = 0;  R.data[1][2] = 0;
R.data[2][0] = 0;  R.data[2][1] = 0;  R.data[2][2] = 1;

accel_cal.set_rotation(R);
```

---

## 📝 Configuration Storage

### Trong NuttX

```cpp
// Save calibration to parameter file
void save_calibration(const Accelerometer& cal, int instance)
{
    char param_name[32];
    
    snprintf(param_name, sizeof(param_name), "CAL_ACC%d_XOFF", instance);
    param_set_float(param_name, cal.offset().x);
    
    snprintf(param_name, sizeof(param_name), "CAL_ACC%d_YOFF", instance);
    param_set_float(param_name, cal.offset().y);
    
    // ... và các parameter khác
    
    param_save();
}
```

### Parameter Names

| Parameter | Mô Tả |
|-----------|-------|
| `CAL_ACC0_XOFF` | Accel 0 X offset |
| `CAL_ACC0_XSCALE` | Accel 0 X scale |
| `CAL_GYRO0_XOFF` | Gyro 0 X offset |
| `CAL_ACC0_ROT` | Rotation enum (0 = no rotation) |

---

## 💡 Best Practices

### 1. Temperature Compensation

```cpp
// Bias drifts với temperature
// Cần calibrate ở nhiều temperature points

struct TempCompensation {
    float ref_temp;       // Reference temperature (°C)
    float bias_coef[3];   // Bias change per °C
};

Vector3f compensate_temp(const Vector3f& bias_ref, 
                         float current_temp,
                         const TempCompensation& comp)
{
    float delta_t = current_temp - comp.ref_temp;
    return Vector3f(
        bias_ref.x + comp.bias_coef[0] * delta_t,
        bias_ref.y + comp.bias_coef[1] * delta_t,
        bias_ref.z + comp.bias_coef[2] * delta_t
    );
}
```

### 2. Validation

```cpp
// Sau calibration, verify kết quả
bool validate_accel_calibration(const Accelerometer& cal)
{
    // Để yên, đo 100 samples
    Vector3f sum(0, 0, 0);
    for (int i = 0; i < 100; i++) {
        sum = sum + cal.correct(read_accel_raw());
        usleep(1000);
    }
    Vector3f avg = sum * 0.01f;
    
    // Magnitude should be ~9.81
    float mag = avg.norm();
    return fabsf(mag - GRAVITY_MSS) < 0.1f;  // ±0.1 m/s²
}
```

---

## 🔗 Liên Kết

- **Usage:** `/apps/uav/sensors_app/`
- **Math Library:** `/apps/uav/lib/mathlib/`
- **Sensor Processing:** `/apps/uav/lib/sensor_processing/`
