# lib/sensor_processing - Multi-IMU Fusion

## 📋 Tổng Quan

Module `sensor_processing` chứa thuật toán **kết hợp dữ liệu từ nhiều IMU** để:
- Tăng độ chính xác qua redundancy
- Phát hiện và loại trừ sensor lỗi
- Cung cấp fault-tolerant sensing cho UAV

---

## 📁 Files

```
sensor_processing/
├── Makefile
├── imu_fusion.hpp    # API và định nghĩa
└── imu_fusion.cpp    # Implementation
```

---

## 🎯 Tại Sao Cần Multi-IMU Fusion?

### Vấn Đề Với Single IMU

```
Single IMU fail → Mất điều khiển → Crash

┌──────────┐
│   IMU    │────X────▶ No data → CRASH!
└──────────┘
```

### Giải Pháp: Redundancy

```
4× IMU với voting/fusion → Tolerant 1 failure

┌──────────┐
│   IMU 0  │───────┐
└──────────┘       │
┌──────────┐       ▼    ┌─────────────┐
│   IMU 1  │──────▶│    │   FUSION    │────▶ Fused output
└──────────┘       │    │   (voting)  │
┌──────────┐       │    └─────────────┘
│   IMU 2  │───────┤          │
└──────────┘       │          │ Detects outlier
┌──────────┐       │          ▼
│   IMU 3  │───X───┘     Exclude IMU 3
└──────────┘
```

---

## 🔧 Các Chế Độ Fusion

### 1. VOTING Mode (Mặc định)

**Ý tưởng:** Dùng **median** - giá trị giữa khi sort. Robust với 1 sensor lỗi.

```
Giả sử 4 IMU đo gyro_x:
  IMU 0: 0.52 rad/s
  IMU 1: 0.51 rad/s
  IMU 2: 9.99 rad/s  ← FAULTY!
  IMU 3: 0.50 rad/s

Sorted: [0.50, 0.51, 0.52, 9.99]
              ↑    ↑
            median = (0.51 + 0.52) / 2 = 0.515

Output: 0.515 rad/s  ✓ Correct!
```

**Ưu điểm:**
- Tự động loại bỏ outlier
- Không cần biết trước sensor nào lỗi
- Simple, deterministic

**Nhược điểm:**
- Chỉ robust với 1 sensor lỗi (với 4 sensors)
- Không tận dụng được accuracy của sensor tốt

### 2. WEIGHTED Mode

**Ý tưởng:** Average có trọng số dựa trên **noise estimate** của từng sensor.

```
                    ∑ (weight_i × value_i)
Fused output = ──────────────────────────────
                       ∑ weight_i

Where: weight_i = 1 / noise_variance_i

Sensor với noise thấp → weight cao → đóng góp nhiều hơn
```

**Ví dụ:**

```
IMU 0: value=0.50, noise=0.01  → weight = 100
IMU 1: value=0.52, noise=0.02  → weight = 50
IMU 2: value=0.51, noise=0.05  → weight = 20
IMU 3: value=0.49, noise=0.01  → weight = 100

Fused = (100×0.50 + 50×0.52 + 20×0.51 + 100×0.49) / 270
      = 0.502 rad/s
```

**Ưu điểm:**
- Tận dụng tối đa accuracy
- Noise reduction

**Nhược điểm:**
- Cần estimate noise online
- Sensor lỗi có thể có low noise → high weight

### 3. PRIMARY Mode

**Ý tưởng:** Dùng 1 IMU chính, chỉ switch khi lỗi.

```
Normal:   IMU_PRIMARY → output
On fail:  IMU_PRIMARY_BACKUP → output
On fail:  IMU_BACKUP_1 → output
...
```

**Ưu điểm:**
- Deterministic behavior
- Consistent response

**Nhược điểm:**
- Không tận dụng redundancy
- Delay khi switch

---

## 📊 Data Structures

### ImuData (Input)

```cpp
struct ImuData {
    float gyro[3];          // rad/s
    float accel[3];         // m/s²
    float temperature;      // °C
    uint64_t timestamp_us;  // Capture time
    uint8_t instance;       // 0-3
    bool valid;             // Data OK?
};
```

### FusedImuData (Output)

```cpp
struct FusedImuData {
    float gyro[3];          // Fused gyro
    float accel[3];         // Fused accel
    float temperature;      // Average temp
    uint64_t timestamp_us;  // Fusion time
    uint8_t num_imus_used;  // 1-4
    uint8_t healthy_mask;   // Bitmask: 0b1011 = IMU 0,1,3 OK
    bool valid;             // Fusion OK?
};
```

### ImuStatus (Per-IMU)

```cpp
struct ImuStatus {
    bool present;           // IMU exists?
    bool functional;        // IMU working?
    bool selected;          // Used in fusion?
    uint32_t fault_count;   // Consecutive faults
    uint32_t total_samples; // Total received
    uint32_t error_samples; // Total errors
    float noise_estimate;   // Estimated noise
    float weight;           // Weight in fusion
};
```

---

## 🔧 API

### Initialization

```cpp
#include <uav/lib/sensor_processing/imu_fusion.hpp>

using namespace uav::sensor_processing;

ImuFusion fusion;

// Init với 4 IMUs
int ret = fusion.init(4);
if (ret != 0) {
    // Error
}

// Set mode
fusion.set_mode(FusionMode::VOTING);  // Default
```

### Input Data

```cpp
// Feed samples từ mỗi IMU
ImuData samples[4];
for (int i = 0; i < 4; i++) {
    samples[i].gyro[0] = raw_gyro_x[i];
    samples[i].gyro[1] = raw_gyro_y[i];
    samples[i].gyro[2] = raw_gyro_z[i];
    samples[i].accel[0] = raw_accel_x[i];
    samples[i].accel[1] = raw_accel_y[i];
    samples[i].accel[2] = raw_accel_z[i];
    samples[i].timestamp_us = now;
    samples[i].instance = i;
    samples[i].valid = true;
}

fusion.update(samples, 4);
```

### Get Output

```cpp
FusedImuData fused;
bool ok = fusion.get_fused_data(&fused);

if (ok && fused.valid) {
    // Use fused.gyro[0..2], fused.accel[0..2]
    
    // Check how many IMUs used
    int num = fused.num_imus_used;
    if (num < 3) {
        // Degraded redundancy warning
    }
}
```

### Query Status

```cpp
// Status của từng IMU
for (int i = 0; i < 4; i++) {
    ImuStatus status = fusion.get_imu_status(i);
    
    printf("IMU %d: present=%d functional=%d selected=%d\n",
           i, status.present, status.functional, status.selected);
    printf("        faults=%d weight=%.2f\n",
           status.fault_count, status.weight);
}

// Healthy mask
uint8_t mask = fusion.get_healthy_mask();
// 0b1111 = all OK
// 0b1011 = IMU 2 failed
```

---

## 🔍 Fault Detection

### Algorithm

```
For each sample:
  1. Compute median of all valid IMUs
  2. For each IMU:
     - deviation = |value - median|
     - if deviation > threshold:
         fault_count++
       else:
         fault_count = 0
     - if fault_count > N:
         mark IMU as FAILED
```

### Thresholds

```cpp
// Gyro fault threshold (rad/s)
CONFIG_UAV_IMU_FAULT_THRESHOLD_GYRO = 0.1  // ~6°/s

// Accel fault threshold (m/s²)
CONFIG_UAV_IMU_FAULT_THRESHOLD_ACCEL = 2.0  // ~0.2g

// Consecutive faults to fail
CONFIG_UAV_IMU_FAULT_COUNT_THRESHOLD = 10
```

### Ví Dụ Fault Detection

```
Sample 1: IMU2 gyro_x = 0.8 (median = 0.5)
          deviation = 0.3 > 0.1 → fault_count[2] = 1

Sample 2: IMU2 gyro_x = 0.9 (median = 0.5)
          deviation = 0.4 > 0.1 → fault_count[2] = 2

...

Sample 10: fault_count[2] = 10 ≥ threshold
           → IMU2 marked FAILED
           → Excluded from fusion
```

---

## 📐 Noise Estimation

Online estimation cho WEIGHTED mode:

```cpp
// Exponential moving average của variance
void update_noise_estimate(int imu, float value, float median)
{
    float deviation = value - median;
    float squared = deviation * deviation;
    
    // EMA update: α = 0.01
    noise_var[imu] = 0.99f * noise_var[imu] + 0.01f * squared;
    
    // Weight = 1/variance (inverse)
    weight[imu] = 1.0f / (noise_var[imu] + 1e-6f);
}
```

---

## 💡 Best Practices

### 1. Mode Selection

```cpp
// Startup: use VOTING (robust)
fusion.set_mode(FusionMode::VOTING);

// After calibration: use WEIGHTED (accurate)
if (calibration_done && all_imus_stable) {
    fusion.set_mode(FusionMode::WEIGHTED);
}

// Degraded: fallback to PRIMARY
if (healthy_count < 3) {
    fusion.set_mode(FusionMode::PRIMARY);
    fusion.set_primary_imu(best_imu);
}
```

### 2. Graceful Degradation

```cpp
void handle_imu_health() {
    int healthy = __builtin_popcount(fusion.get_healthy_mask());
    
    if (healthy >= 4) {
        // All good - full redundancy
    } else if (healthy >= 3) {
        // 1 fail - still robust
        warn("IMU redundancy degraded");
    } else if (healthy >= 2) {
        // 2 fail - reduced capability
        warn("Low IMU redundancy - land soon");
    } else if (healthy >= 1) {
        // 3 fail - critical
        warn("Critical: Single IMU - land immediately");
    } else {
        // All fail - emergency
        trigger_failsafe();
    }
}
```

### 3. Temperature Compensation

```cpp
// Bias drifts với temperature
// Cần compensate trước khi fusion
float gyro_bias[3] = {
    base_bias[0] + temp_coef[0] * (temp - ref_temp),
    base_bias[1] + temp_coef[1] * (temp - ref_temp),
    base_bias[2] + temp_coef[2] * (temp - ref_temp)
};

sample.gyro[0] -= gyro_bias[0];
sample.gyro[1] -= gyro_bias[1];
sample.gyro[2] -= gyro_bias[2];
```

---

## 📊 Performance

| Operation | Time (Cortex-M7 @ 480MHz) |
|-----------|---------------------------|
| Median (4 values) | ~50 ns |
| update() (4 IMUs) | ~3 µs |
| Fault check | ~1 µs |
| Weight update | ~500 ns |

---

## 🔗 Liên Kết

- **Usage:** `/apps/uav/sensors_app/`
- **Health Monitor:** `/apps/uav/lib/health/`
- **DSP Filters:** `/apps/uav/lib/dsp/`
- **Math:** `/apps/uav/lib/mathlib/`
