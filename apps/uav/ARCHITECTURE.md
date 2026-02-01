# UAV Avionics Stack - Time-Triggered Architecture

## Tổng Quan

Đây là hệ thống avionics cho UAV, được thiết kế theo kiến trúc **Time-Triggered** (TTA) trên NuttX RTOS. Kiến trúc này lấy cảm hứng từ:

- **PX4**: Cấu trúc module và uORB messaging
- **VectorNav**: Pipeline xử lý sensor và timing deterministic

### Đặc Điểm Chính

| Đặc điểm | Mô tả |
|----------|-------|
| **Master Rate** | 8 kHz - tần số tick cao nhất của hệ thống |
| **Deterministic** | Mọi task đều sync với master tick |
| **Multi-rate** | FAST (8kHz), MEDIUM (1kHz), SLOW (100Hz) |
| **DMA** | Sử dụng DMA cho SPI để giảm CPU load |
| **Redundancy** | 4 IMU với voting/fusion |

---

## Kiến Trúc Hệ Thống

### Sơ Đồ Tổng Quan

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         MASTER TIMEBASE (8 kHz)                              │
│                    Hardware Timer → Signal Handler                           │
└──────────────────────────────┬──────────────────────────────────────────────┘
                               │
         ┌─────────────────────┼─────────────────────┐
         │                     │                     │
         ▼                     ▼                     ▼
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│  FAST DOMAIN    │  │  MEDIUM DOMAIN  │  │  SLOW DOMAIN    │
│    (8 kHz)      │  │    (1 kHz)      │  │   (100 Hz)      │
├─────────────────┤  ├─────────────────┤  ├─────────────────┤
│ • IMU DMA       │  │ • EKF Predict   │  │ • Baro Read     │
│ • Gyro Integ    │  │ • Sensor Fusion │  │ • Mag Process   │
│ • Delta Angle   │  │ • Health Check  │  │ • GPS Fusion    │
└─────────────────┘  └─────────────────┘  └─────────────────┘
```

### Data Flow

```
  ┌─────────┐    DMA      ┌─────────────┐         ┌─────────────┐
  │ ICM4268 │────────────▶│ Double      │────────▶│ Calibration │
  │ x4 SPI  │  8kHz burst │ Buffer      │         │ + Filter    │
  └─────────┘             └─────────────┘         └──────┬──────┘
                                                         │
                                                         ▼
                                              ┌─────────────────┐
                                              │   IMU Fusion    │
                                              │ (Voting/Weight) │
                                              └────────┬────────┘
                                                       │
                                                       ▼
                                              ┌─────────────────┐
                                              │ sensor_combined │
                                              │   (uORB topic)  │
                                              └────────┬────────┘
                                                       │
                                                       ▼
                                              ┌─────────────────┐
                                              │      EKF        │
                                              └─────────────────┘
```

---

## Cấu Trúc Thư Mục

```
apps/uav/
├── lib/                           # Shared libraries
│   ├── platform/                  # Platform abstraction
│   │   ├── hrt.h/c               # High-resolution timer
│   │   ├── timebase.h/c          # Master timebase
│   │   └── spi_config.h          # SPI configuration
│   │
│   ├── utils/                     # Utility libraries
│   │   ├── ringbuf.hpp           # Lock-free ring buffer
│   │   ├── dma_buffer.hpp        # DMA double buffer
│   │   ├── debug.hpp             # Debug utilities
│   │   └── critical.hpp          # Critical sections
│   │
│   ├── dsp/                       # Digital signal processing
│   │   └── filters.hpp           # Lowpass, notch, median filters
│   │
│   ├── health/                    # Health monitoring
│   │   ├── health_monitor.hpp
│   │   └── health_monitor.cpp
│   │
│   ├── sensor_processing/         # Sensor data processing
│   │   ├── imu_fusion.hpp        # Multi-IMU fusion
│   │   └── imu_fusion.cpp
│   │
│   ├── calibration/               # Sensor calibration
│   │   └── sensor_calibration.hpp/cpp
│   │
│   ├── mathlib/                   # Math utilities
│   │   └── matrix.hpp, quaternion.hpp
│   │
│   └── drivers_framework/         # Driver base classes
│       └── device_id.hpp
│
├── drivers/                       # Hardware drivers
│   ├── spi/                       # SPI base class
│   │   ├── spi_device.hpp/cpp
│   │   └── spi_types.hpp
│   │
│   ├── imu/                       # IMU drivers
│   │   └── icm42688p/
│   │       ├── icm42688p.hpp/cpp
│   │       └── icm42688p_regs.h
│   │
│   ├── baro/                      # Barometer drivers
│   ├── mag/                       # Magnetometer drivers
│   └── gps/                       # GPS drivers
│
├── uorb/                          # Message bus
│   ├── uorb.hpp/cpp              # uORB implementation
│   ├── orb_defines.hpp           # Macros và definitions
│   ├── topics.cpp                # Topic definitions
│   └── topics/                    # Topic message structs
│       ├── sensor_imu.hpp
│       ├── sensor_combined.hpp
│       ├── system_status.hpp
│       ├── vehicle_attitude.hpp
│       └── ...
│
├── sensors_app/                   # Sensors application
│   ├── sensors_main.cpp          # Main entry point
│   └── Makefile
│
├── estimator_app/                 # EKF estimator
│   └── estimator_main.cpp
│
└── state_app/                     # State machine
    └── state_main.cpp
```

---

## Master Timebase

### Mục Đích

Cung cấp tick chính xác cho toàn hệ thống, đảm bảo:
- Deterministic timing
- Multi-rate synchronization
- Minimal jitter

### Domains

| Domain | Tần Số | Divider | Mục Đích |
|--------|--------|---------|----------|
| FAST | 8 kHz | 1 | IMU acquisition, gyro integration |
| MEDIUM | 1 kHz | 8 | EKF predict, sensor fusion |
| SLOW | 100 Hz | 80 | Baro/Mag/GPS, covariance update |
| ASYNC | Event | - | GPS parsing, telemetry |

### API

```c
/* Khởi tạo */
int timebase_init(void);

/* Đợi tick của domain */
int timebase_wait_domain(timebase_domain_t domain);

/* Lấy timestamp */
uint64_t timebase_get_timestamp(void);

/* Signal async event */
void timebase_signal_async(void);
```

---

## IMU Fusion

### Chế Độ Fusion

1. **VOTING** (Mặc định)
   - Sử dụng median của tất cả IMU
   - Robust với 1 sensor bị lỗi
   - Không cần calibration weight

2. **WEIGHTED**
   - Average có trọng số theo noise estimate
   - Chính xác hơn khi sensors khác nhau về chất lượng
   - Cần thời gian để estimate noise

3. **PRIMARY**
   - Dùng 1 IMU chính
   - Fallback khi primary lỗi
   - Latency thấp nhất

### Fault Detection

- So sánh mỗi IMU với median
- IMU lệch > threshold được đánh dấu faulty
- Sau N samples liên tiếp → disable IMU

---

## Health Monitoring

### Health Levels

| Level | Mô Tả | Hành Động |
|-------|-------|-----------|
| NOMINAL | Tất cả OK | Bay bình thường |
| DEGRADED | Mất redundancy | Cảnh báo, có thể bay |
| CRITICAL | Tối thiểu chức năng | Nên hạ cánh |
| FAILSAFE | Unsafe | Chỉ giữ thăng bằng |

### Components Được Monitor

- Master tick
- IMU 0-3
- Barometer
- Magnetometer
- GPS
- EKF
- Logger

---

## Filters (DSP)

### LowPassFilter2p

Butterworth 2nd order lowpass filter:
- Đáp ứng phẳng trong passband
- 12 dB/octave rolloff
- Dùng cho gyro (100 Hz cutoff) và accel (50 Hz cutoff)

### NotchFilter

Loại bỏ frequency cụ thể (motor vibration):
- Configurable notch frequency
- Adjustable bandwidth

### MedianFilter

Loại bỏ spikes:
- Không làm mờ edges như lowpass
- Dùng khi có noise impulsive

---

## Ring Buffers

### SPSC Ring Buffer

Lock-free Single Producer Single Consumer:
- Zero-copy khi có thể
- Power-of-2 size
- Memory barriers cho correctness

### DMA Double Buffer

Zero-copy DMA transfers:
- Hai buffers được swap
- DMA ghi vào một, CPU đọc buffer kia
- Timestamp capture ngay khi DMA complete

---

## Cách Sử Dụng

### Build

```bash
cd nuttxspace/nuttx
make menuconfig  # Enable CONFIG_UAV
make -j8
```

### Chạy

```bash
# Trong NuttX shell
sensors start     # Khởi động sensors
sensors status    # Xem trạng thái
sensors timebase  # Xem master timebase
sensors health    # Xem health monitor
sensors fusion    # Xem IMU fusion
sensors stop      # Dừng
```

---

## Cấu Hình Kconfig

```kconfig
# Master timebase
CONFIG_UAV_MASTER_TICK_HZ=8000
CONFIG_UAV_MEDIUM_DIVIDER=8
CONFIG_UAV_SLOW_DIVIDER=80

# Sensors
CONFIG_UAV_NUM_IMUS=4
CONFIG_UAV_IMU_RATE_HZ=1000

# Filters
CONFIG_UAV_GYRO_CUTOFF_HZ=100
CONFIG_UAV_ACCEL_CUTOFF_HZ=50

# Fusion mode: 0=VOTING, 1=WEIGHTED, 2=PRIMARY
CONFIG_UAV_IMU_FUSION_MODE=0

# Health monitoring
CONFIG_UAV_HEARTBEAT_TIMEOUT_MS=50
CONFIG_UAV_MISSED_BEATS_WARN=3
CONFIG_UAV_MISSED_BEATS_FAIL=10
```

---

## Roadmap

### Phase 1 (Hiện tại)
- ✅ Master timebase
- ✅ Ring buffers
- ✅ DSP filters
- ✅ IMU fusion
- ✅ Health monitoring
- ⏳ Integration với real ICM42688P

### Phase 2
- ❌ DMA SPI transfers
- ❌ Hardware timestamps
- ❌ Coning/sculling compensation

### Phase 3
- ❌ Full EKF integration
- ❌ Baro/Mag/GPS fusion
- ❌ Telemetry output

---

## Tham Khảo

- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
- [NuttX Documentation](https://nuttx.apache.org/docs/latest/)
- [VectorNav VN-100](https://www.vectornav.com/products/detail/vn-100)

---

*Tài liệu này được tạo tự động. Cập nhật lần cuối: 2026-02-01*
