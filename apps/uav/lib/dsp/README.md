# lib/dsp - Digital Signal Processing

## 📋 Tổng Quan

Module `dsp` cung cấp các **bộ lọc số** (digital filters) để xử lý tín hiệu từ sensors. Mục đích chính:

1. **Loại bỏ nhiễu cao tần** (lowpass filter)
2. **Loại bỏ frequency cụ thể** (notch filter) - motor vibration
3. **Loại bỏ spikes/outliers** (median filter)
4. **Downsampling có filter** (decimation)

---

## 📁 Files

```
dsp/
├── Makefile
├── filters.hpp     # Tất cả filter implementations
└── README.md       # File này
```

---

## 🔊 LowPassFilter2p - Biquad Lowpass

### Mô Tả

Bộ lọc thông thấp **bậc 2 Butterworth** (biquad):
- **Đáp ứng phẳng** trong passband
- **Rolloff:** 12 dB/octave (40 dB/decade)
- **Phase shift:** 90° tại cutoff frequency

### Sơ Đồ Đáp Ứng

```
Gain (dB)
   0 ──────────────────┐
                       │
  -3 ─ ─ ─ ─ ─ ─ ─ ─ ─ ┼─ ─ ─ ─ ─ cutoff
                       │\
  -12 ─ ─ ─ ─ ─ ─ ─ ─ ─┼─\─ ─ ─ ─ 1 octave
                       │  \
  -24 ─ ─ ─ ─ ─ ─ ─ ─ ─┼───\─ ─ ─ 2 octaves
                       │    \
      ─────────────────┴──────────────▶ Frequency
                     fc           fs/2
```

### API

```cpp
#include <uav/lib/dsp/filters.hpp>

using namespace uav::dsp;

// Tạo filter
LowPassFilter2p filter;

// Cấu hình: sample_freq, cutoff_freq
filter.set_cutoff_frequency(1000.0f, 100.0f);  // fs=1kHz, fc=100Hz

// Áp dụng filter (trong loop)
float raw_gyro = get_gyro_reading();
float filtered_gyro = filter.apply(raw_gyro);
```

### Chọn Cutoff Frequency

| Sensor | Sample Rate | Cutoff | Lý do |
|--------|-------------|--------|-------|
| Gyro | 1000 Hz | 80-100 Hz | Loại nhiễu, giữ dynamics |
| Accel | 1000 Hz | 30-50 Hz | Rung động motor |
| Baro | 50 Hz | 5 Hz | Nhiễu khí động học |

### Implementation

```cpp
// Biquad Direct Form II Transposed
// y[n] = b0*x[n] + d1
// d1 = b1*x[n] - a1*y[n] + d2
// d2 = b2*x[n] - a2*y[n]

float apply(float sample) {
    float out = m_b0 * sample + m_delay1;
    m_delay1 = m_b1 * sample - m_a1 * out + m_delay2;
    m_delay2 = m_b2 * sample - m_a2 * out;
    return out;
}
```

---

## 🔕 NotchFilter - Loại Bỏ Frequency Cụ Thể

### Mô Tả

Bộ lọc **notch** (band-reject) để loại bỏ frequency cụ thể, thường dùng cho:
- **Motor vibration** (400-600 Hz typical)
- **ESC switching noise**
- **Resonance frequencies**

### Sơ Đồ Đáp Ứng

```
Gain (dB)
   0 ────────────────┬───────────────────
                     │       
                     │       
  -20 ─ ─ ─ ─ ─ ─ ─ ─│─ ─ ─ ─ ─ ─ ─ ─ ─
                    \│/      
  -40 ─ ─ ─ ─ ─ ─ ─ ─V─ ─ ─ ─ ─ ─ ─ ─ ─
                     │       
      ───────────────┴───────────────────▶ Frequency
                  f_notch
```

### API

```cpp
#include <uav/lib/dsp/filters.hpp>

using namespace uav::dsp;

NotchFilter notch;

// Cấu hình: sample_freq, notch_freq, Q_factor
notch.set_notch(1000.0f, 400.0f, 2.0f);

// Q càng cao → notch càng hẹp
// Q = 0.5: wide notch (affect more frequencies)
// Q = 5.0: narrow notch (precise removal)

// Áp dụng
float clean = notch.apply(vibrating_signal);
```

### Cascading Multiple Notches

```cpp
// Nhiều harmonic của motor frequency
NotchFilter motor_fundamental;  // 400 Hz
NotchFilter motor_2nd;          // 800 Hz
NotchFilter motor_3rd;          // 1200 Hz

motor_fundamental.set_notch(fs, 400.0f, 2.0f);
motor_2nd.set_notch(fs, 800.0f, 2.0f);
motor_3rd.set_notch(fs, 1200.0f, 2.0f);

// Cascade
float out = motor_fundamental.apply(in);
out = motor_2nd.apply(out);
out = motor_3rd.apply(out);
```

---

## 📊 MedianFilter - Loại Bỏ Spikes

### Mô Tả

Bộ lọc **median** loại bỏ các giá trị bất thường (outliers/spikes):
- **Robust** với single-sample spikes
- **Giữ edges** tốt hơn lowpass
- **Latency** = (window_size - 1) / 2 samples

### API

```cpp
#include <uav/lib/dsp/filters.hpp>

using namespace uav::dsp;

// Template parameter: window size (must be odd)
MedianFilter<5> median;

// Áp dụng
float clean = median.apply(noisy_value);
```

### So Sánh với Lowpass

```
Input:    1  2  3  100  4  5  6
                    ▲
                  spike

Lowpass output:  ~1.5  ~2  ~15  ~20  ~8  ~5  ~5.5  (spike spreads)
Median output:    1    2   3    4   4   5   6     (spike removed)
```

### Use Cases

- **GPS velocity:** Occasional spikes
- **Baro altitude:** Sensor glitches
- **Sonar distance:** False readings

---

## ⬇️ DecimationFilter - Downsample

### Mô Tả

Bộ lọc **decimation** để giảm sample rate:
1. **Lowpass filter** để chống aliasing
2. **Downsample** bằng cách bỏ samples

### API

```cpp
#include <uav/lib/dsp/filters.hpp>

using namespace uav::dsp;

// Decimation factor: 8 (1000 Hz → 125 Hz)
DecimationFilter<8> decimator;

decimator.set_cutoff(1000.0f, 50.0f);  // Anti-aliasing at 50 Hz

// Input @ 1000 Hz
for (int i = 0; i < 8; i++) {
    float sample = get_sample();
    bool output_ready = decimator.apply(sample);
    
    if (output_ready) {
        float decimated = decimator.get_output();  // @ 125 Hz
    }
}
```

---

## ⚡ Performance

### CPU Usage (Cortex-M7 @ 400 MHz)

| Filter | Per Sample | @ 1 kHz |
|--------|------------|---------|
| LowPass2p | ~50 ns | ~0.005% |
| Notch | ~50 ns | ~0.005% |
| Median<5> | ~100 ns | ~0.01% |
| Median<7> | ~200 ns | ~0.02% |

### Memory Usage

| Filter | Size |
|--------|------|
| LowPass2p | 28 bytes |
| Notch | 28 bytes |
| Median<5> | 24 bytes |

---

## 💡 Best Practices

### 1. Set Cutoff Trước Khi Apply

```cpp
// ❌ Sai - cutoff chưa set
LowPassFilter2p filter;
filter.apply(sample);  // Bypass mode!

// ✅ Đúng
LowPassFilter2p filter;
filter.set_cutoff_frequency(1000.0f, 100.0f);
filter.apply(sample);
```

### 2. Reset Khi Đổi Config

```cpp
// Reset state khi thay đổi cutoff
filter.reset();
filter.set_cutoff_frequency(new_fs, new_fc);
```

### 3. Separate Filters cho Mỗi Axis

```cpp
// ✅ Mỗi axis cần filter riêng
LowPassFilter2p gyro_filter_x;
LowPassFilter2p gyro_filter_y;
LowPassFilter2p gyro_filter_z;

// ❌ Không dùng chung 1 filter cho nhiều axis
```

### 4. Latency Consideration

```
Lowpass 2nd order: ~2 samples delay
Median 5: 2 samples delay
Cascaded filters: delays add up
```

---

## 🔗 Liên Kết

- **Usage:** `/apps/uav/sensors_app/sensors_main.cpp`
- **Theory:** [Butterworth Filter](https://en.wikipedia.org/wiki/Butterworth_filter)
- **mathlib filters:** `/apps/uav/lib/mathlib/filters.hpp` (simple IIR)
