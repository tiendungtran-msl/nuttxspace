# UAV Avionics Stack trên NuttX RTOS

> Hệ thống phần mềm điều khiển UAV theo kiến trúc **multi-app / time-triggered**, viết cho **NuttX RTOS** trên vi điều khiển **STM32H743**. Lấy cảm hứng từ PX4 Autopilot và VectorNav, được viết lại hoàn toàn từ đầu.

---

## Mục lục

1. [Tổng quan](#1-tổng-quan)
2. [Yêu cầu hệ thống](#2-yêu-cầu-hệ-thống)
3. [Kiến trúc hệ thống](#3-kiến-trúc-hệ-thống)
4. [Cấu trúc thư mục](#4-cấu-trúc-thư-mục)
5. [Các ứng dụng (Apps)](#5-các-ứng-dụng-apps)
6. [Hệ thống uORB](#6-hệ-thống-uorb)
7. [Hardware Drivers](#7-hardware-drivers)
8. [Triển khai (Build & Flash)](#8-triển-khai-build--flash)
9. [Chạy trên phần cứng](#9-chạy-trên-phần-cứng)
10. [Calibration cảm biến](#10-calibration-cảm-biến)
11. [Cấu hình Kconfig](#11-cấu-hình-kconfig)
12. [Ràng buộc kỹ thuật](#12-ràng-buộc-kỹ-thuật)
13. [Roadmap](#13-roadmap)
14. [Tham khảo](#14-tham-khảo)

---

## 1. Tổng quan

### Triết lý thiết kế

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    NGUYÊN TẮC THIẾT KẾ FLIGHT-CRITICAL                      │
├─────────────────────────────────────────────────────────────────────────────┤
│ 1. FAULT ISOLATION    : Mỗi app là task riêng — crash 1 không kill system   │
│ 2. DETERMINISM        : Timing có thể dự đoán, jitter < 50 µs              │
│ 3. NO DYNAMIC ALLOC   : Không malloc/new trong real-time loop              │
│ 4. LOOSE COUPLING     : Giao tiếp chỉ qua uORB, không gọi hàm trực tiếp   │
│ 5. SINGLE OWNERSHIP   : Mỗi hardware chỉ có 1 owner duy nhất              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Hardware target

| Thành phần   | Chi tiết                                    |
|--------------|---------------------------------------------|
| **MCU**      | STM32H743 — Cortex-M7 @ 480 MHz, 1 MB RAM  |
| **IMU**      | ICM-42688-P × 4 (SPI1, redundant)          |
| **Magnetometer** | BMM150 (I2C1)                           |
| **Barometer** | MS5611 (I2C1) — *planned*                 |
| **GPS**      | u-blox (UART) — *planned*                  |

---

## 2. Yêu cầu hệ thống

### Phần mềm phát triển

| Công cụ | Phiên bản tối thiểu | Ghi chú |
|---------|---------------------|---------|
| `arm-none-eabi-gcc` | 10.x | Cross-compiler cho Cortex-M7 |
| `make` | 4.x | GNU Make |
| `cmake` | 3.16+ | Tùy board config |
| NuttX | mainline / 12.x | Apache NuttX RTOS |
| Python | 3.8+ | Dùng cho script tiện ích |
| ST-Link / OpenOCD | mới nhất | Flashing và debugging |

### Cách cài đặt cross-compiler (Ubuntu/Debian)

```bash
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi \
     make git python3 python3-pip kconfig-frontends
```

### Clone NuttX workspace

```bash
mkdir nuttxspace && cd nuttxspace
git clone https://github.com/apache/nuttx.git nuttx
git clone https://github.com/apache/nuttx-apps.git apps

# Copy UAV stack vào apps
cp -r <path_to_uav_repo>/apps/uav  apps/uav
```

---

## 3. Kiến trúc hệ thống

### Sơ đồ tổng quan

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         MASTER TIMEBASE (8 kHz)                             │
│                    Hardware Timer → Signal Handler                          │
└──────────────────────────────┬──────────────────────────────────────────────┘
                               │
         ┌─────────────────────┼─────────────────────┐
         │                     │                     │
         ▼                     ▼                     ▼
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│  FAST DOMAIN    │  │  MEDIUM DOMAIN  │  │  SLOW DOMAIN    │
│    (8 kHz)      │  │    (1 kHz)      │  │   (100 Hz)      │
├─────────────────┤  ├─────────────────┤  ├─────────────────┤
│ • IMU DMA read  │  │ • EKF predict   │  │ • Baro read     │
│ • Gyro integ.   │  │ • Sensor fusion │  │ • Mag process   │
│ • Delta angle   │  │ • Health check  │  │ • GPS fusion    │
└─────────────────┘  └─────────────────┘  └─────────────────┘
         │                     │                     │
         ▼                     ▼                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                               uORB MESSAGE BUS                              │
│  ┌──────────────┐ ┌───────────────┐ ┌────────────────┐ ┌───────────────┐   │
│  │ sensor_imu   │ │sensor_combined│ │vehicle_attitude│ │system_status  │   │
│  └──────────────┘ └───────────────┘ └────────────────┘ └───────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
         │                     │                     │
         ▼                     ▼                     ▼
┌───────────────┐   ┌───────────────┐   ┌───────────────┐   ┌───────────────┐
│  sensors_app  │   │ estimator_app │   │   state_app   │   │ telemetry_app │
│   Prio: 250   │   │   Prio: 240   │   │   Prio: 220   │   │   Prio: 100   │
│   @ 1 kHz     │   │   @ 250 Hz    │   │   @ 50 Hz     │   │   @ 10 Hz     │
└───────────────┘   └───────────────┘   └───────────────┘   └───────────────┘
```

### Timing budget (1 kHz cycle — 1000 µs)

| Công việc | Thời gian |
|-----------|-----------|
| SPI DMA transfer (4 IMU song song) | ~100 µs |
| Calibration apply | ~20 µs |
| Lowpass filtering | ~30 µs |
| IMU fusion (voting/weighted) | ~50 µs |
| uORB publish | ~20 µs |
| **Tổng sử dụng** | **~220 µs (22% CPU)** |
| **Margin còn lại** | **~780 µs (78% free)** |

### Data flow

```
[ICM42688P ×4] ──DMA──▶ [Double Buffer] ──▶ [Calib + Filter]
                                                      │
                                                      ▼
                                             [IMU Fusion]
                                          (Voting / Weighted)
                                                      │
                                                      ▼
                                          [sensor_combined topic]
                                                      │
                                                      ▼
                                               [EKF predict]
                                                      │
                                                      ▼
                                          [vehicle_attitude topic]
```

---

## 4. Cấu trúc thư mục

```
apps/uav/
├── README.md                  # File này
├── ARCHITECTURE.md            # Tài liệu kiến trúc chi tiết
├── TELEMETRY_ARCHITECTURE.md  # Tài liệu telemetry
├── Makefile                   # Build tất cả sub-components
├── Kconfig                    # Menu cấu hình cho menuconfig
│
├── lib/                       # [SHARED] Thư viện dùng chung
│   ├── platform/              # Platform abstraction layer
│   │   ├── hrt.h/c           # High-resolution timer
│   │   ├── timebase.h/c      # Master timebase (8 kHz)
│   │   └── spi_config.h      # SPI pin & CS cấu hình
│   ├── utils/                 # Utility
│   │   ├── ringbuf.hpp       # Lock-free SPSC ring buffer
│   │   └── critical.hpp      # Critical section helpers
│   ├── dsp/                   # Digital Signal Processing
│   │   └── filters.hpp       # Butterworth 2p, Notch, Median
│   ├── health/                # Health monitoring
│   │   ├── health_monitor.hpp
│   │   └── health_monitor.cpp
│   ├── sensor_processing/     # Xử lý cảm biến
│   │   ├── imu_fusion.hpp    # Multi-IMU fusion
│   │   └── imu_fusion.cpp
│   ├── calibration/           # Sensor calibration objects
│   │   └── sensor_calibration.hpp/cpp
│   └── mathlib/               # Toán học
│       └── matrix.hpp, quaternion.hpp
│
├── drivers/                   # [SHARED] Hardware drivers
│   ├── spi/                   # SPI base class
│   │   └── spi_device.hpp/cpp
│   ├── imu/                   # IMU drivers
│   │   └── icm42688p/
│   │       ├── icm42688p.hpp/cpp
│   │       └── icm42688p_regs.h
│   ├── mag/                   # Magnetometer drivers
│   │   └── bmm150/
│   │       ├── bmm150.hpp/cpp
│   │       └── bmm150_reg.h
│   ├── baro/                  # Barometer (planned)
│   └── gps/                   # GPS (planned)
│
├── uorb/                      # [SHARED] Message bus
│   ├── uorb.hpp/cpp          # uORB API
│   ├── orb_defines.hpp       # Macros & topic IDs
│   ├── topics.cpp            # Topic registration
│   └── topics/               # Message struct definitions
│       ├── sensor_imu.hpp
│       ├── sensor_combined.hpp
│       ├── vehicle_attitude.hpp
│       ├── system_status.hpp
│       └── ...
│
├── sensors_app/               # [APP] Thu thập cảm biến @ 1 kHz
│   ├── Makefile
│   ├── Kconfig
│   ├── sensors_main.cpp
│   └── README.md
│
├── estimator_app/             # [APP] EKF2 state estimator @ 250 Hz
│   ├── Makefile
│   ├── Kconfig
│   ├── estimator_main.cpp
│   └── README.md
│
├── state_app/                 # [APP] State machine & health @ 50 Hz
│   ├── Makefile
│   ├── Kconfig
│   ├── state_main.cpp
│   └── README.md
│
├── telemetry_app/             # [APP] Telemetry output @ 10 Hz
│   ├── Makefile
│   ├── Kconfig
│   ├── telemetry_main.cpp
│   ├── telemetry_packet.h
│   └── telemetry_packet.c
│
└── calib_app/                 # [APP] Interactive sensor calibration
    ├── Makefile
    ├── Kconfig
    └── calib_main.cpp
```

---

## 5. Các ứng dụng (Apps)

### Bảng tổng hợp

| App | Priority | Rate | Stack | Nhiệm vụ | Publishes | Subscribes |
|-----|----------|------|-------|----------|-----------|------------|
| `sensors_app` | 250 | 1 kHz | 8 KB | Driver polling, IMU fusion | `sensor_imu[0-3]`, `sensor_combined`, `system_status` | — |
| `estimator_app` | 240 | 250 Hz | 8 KB | EKF2 attitude estimation | `vehicle_attitude`, `vehicle_local_position` | `sensor_combined` |
| `state_app` | 220 | 50 Hz | 4 KB | State machine, failsafe | `vehicle_state` | `vehicle_*`, `ekf2_status`, `system_status` |
| `telemetry_app` | 100 | 10 Hz | 4 KB | Serial telemetry output | — | `vehicle_*`, `sensor_combined` |
| `calib_app` | — | On-demand | 4 KB | Interactive calibration (NSH) | — | — |

---

### 5.1 `sensors_app`

**Nhiệm vụ:** Thu thập dữ liệu từ 4 IMU ICM-42688-P và BMM150, áp dụng calibration + filtering, thực hiện IMU fusion, publish lên uORB.

**Pipeline xử lý:**

```
ICM42688P[0..3] → read() → apply_calib() → LowPassFilter2p → ImuFusion → sensor_combined
BMM150          → read() → apply_offset() → sensor_mag
```

**Các lệnh NSH:**

```bash
nsh> sensors start      # Khởi động task (SCHED_FIFO, priority 250)
nsh> sensors stop       # Dừng task
nsh> sensors status     # Trạng thái chi tiết (loop count, deadline misses, jitter)
nsh> sensors test       # Chạy self-test trên tất cả IMU
nsh> sensors timebase   # Trạng thái master timebase
nsh> sensors health     # Trạng thái health monitor
nsh> sensors fusion     # Trạng thái IMU fusion (mode, healthy mask)
```

**IMU Fusion modes:**

| Mode | Mô tả | Khi nào dùng |
|------|-------|---------------|
| `VOTING` (0) | Median của tất cả IMU — robust với 1 sensor lỗi | Mặc định |
| `WEIGHTED` (1) | Weighted average theo noise level | Khi cần chính xác hơn |
| `PRIMARY` (2) | IMU chính, fallback nếu lỗi | Latency thấp nhất |

---

### 5.2 `estimator_app`

**Nhiệm vụ:** Chạy Error-State Kalman Filter (EKF2) để ước lượng attitude (quaternion), vận tốc, và vị trí từ dữ liệu IMU/GPS/Baro.

**Pipeline:**

```
sensor_combined → EKF predict (250 Hz) → vehicle_attitude
sensor_gps      → EKF update           → vehicle_local_position
sensor_baro     → EKF update (100 Hz)  → ekf2_status
```

**Các lệnh NSH:**

```bash
nsh> estimator start    # Khởi động estimator task
nsh> estimator stop     # Dừng
nsh> estimator status   # Xem covariance, innovation, trạng thái
```

---

### 5.3 `state_app`

**Nhiệm vụ:** State machine quản lý vòng đời bay, giám sát health tổng, trigger failsafe khi cần.

**Health levels:**

| Level | Mô tả | Hành động |
|-------|-------|-----------|
| `NOMINAL` | Tất cả OK | Bay bình thường |
| `DEGRADED` | Mất redundancy | Cảnh báo, có thể bay |
| `CRITICAL` | Chỉ tối thiểu chức năng | Nên return to home |
| `FAILSAFE` | Unsafe to fly | Motoroff hoặc giữ thăng bằng |

**Các lệnh NSH:**

```bash
nsh> state start
nsh> state stop
nsh> state status       # Xem state machine hiện tại và health
```

---

### 5.4 `telemetry_app`

**Nhiệm vụ:** Subscribe các topic state/sensor, đóng gói thành binary packet và gửi qua UART đến GCS (Ground Control Station).

**각 lệnh NSH:**

```bash
nsh> telemetry start [/dev/ttyS1]   # Khởi động, mặc định /dev/ttyS1
nsh> telemetry stop
nsh> telemetry status               # Xem packet count, baud rate
```

---

### 5.5 `calib_app`

**Nhiệm vụ:** Công cụ calibration cảm biến chạy tương tác trên NSH shell. Yêu cầu `sensors_app` đã chạy trước.

**Lưu ý:** `calib_app` truy cập trực tiếp `g_imu_instance` và `g_mag_instance` được khởi tạo bởi `sensors_app` qua `extern` pointer.

**Các lệnh:**

```bash
nsh> calib gyro         # Calibrate gyro bias (giữ board yên)
nsh> calib accel        # Calibrate accel bias (đặt nằm ngang)
nsh> calib mag          # Calibrate mag hard-iron (xoay 360° trên 3 mặt phẳng)
nsh> calib status       # Xem calibration hiện tại của tất cả sensor
nsh> calib reset        # Reset về mặc định
```

---

## 6. Hệ thống uORB

uORB (Micro Object Request Broker) là pub/sub message bus nội bộ — cơ chế giao tiếp duy nhất giữa các app.

### API cơ bản

```cpp
// Publisher
#include <uav/uorb/uorb.hpp>
#include <uav/uorb/topics/sensor_imu.hpp>

sensor_imu_s msg = {};
uorb::orb_advert_t pub = uorb::orb_advertise(ORB_ID(sensor_imu), &msg);
// ... fill msg ...
uorb::orb_publish(ORB_ID(sensor_imu), pub, &msg);
uorb::orb_unadvertise(pub);  // Khi kết thúc

// Subscriber
int sub = uorb::orb_subscribe(ORB_ID(sensor_imu));
bool updated;
uorb::orb_check(sub, &updated);
if (updated) {
    uorb::orb_copy(ORB_ID(sensor_imu), sub, &msg);
}
uorb::orb_unsubscribe(sub);
```

### Các topics chính

| Topic | Publisher | Subscribers | Nội dung |
|-------|-----------|-------------|---------|
| `sensor_imu[0-3]` | `sensors_app` | `calib_app` | Raw IMU data mỗi sensor |
| `sensor_combined` | `sensors_app` | `estimator_app`, `telemetry_app` | Fused IMU data |
| `vehicle_attitude` | `estimator_app` | `state_app`, `telemetry_app` | Quaternion + angular rate |
| `vehicle_local_position` | `estimator_app` | `state_app`, `telemetry_app` | NED position + velocity |
| `system_status` | `sensors_app` | `state_app`, `telemetry_app` | Health, uptime, jitter |

---

## 7. Hardware Drivers

### ICM-42688-P (IMU) — SPI1

- 4 instance kết nối song song, mỗi instance có CS pin riêng
- ODR cài đặt: 8 kHz, đọc tại 1 kHz
- Gyro range: ±2000 °/s; Accel range: ±16 g
- Hỗ trợ calibration bias + accel scale correction

```cpp
#include <uav/drivers/imu/icm42688p/icm42688p.hpp>

drivers::imu::ICM42688P imu(bus_id, device_id);
imu.init();
ICM42688P::Data data;
imu.read(data);  // data.gyro[3], data.accel[3], data.temperature
```

### BMM150 (Magnetometer) — I2C1

- Địa chỉ I2C: `0x13` (hoặc `0x10`–`0x13` tùy jumper)
- ODR: 20 Hz, range: ±1300 µT (XY), ±2500 µT (Z)
- Hỗ trợ hard-iron offset calibration

```cpp
#include <uav/drivers/mag/bmm150/bmm150.hpp>

struct i2c_master_s *i2c = stm32_i2cbus_initialize(1);
drivers::mag::BMM150 mag(i2c, 0x13);
mag.initialize();
BMM150::Data data;
mag.read(data);  // data.mag[3] in µT
```

### DSP Filters (lib/dsp/filters.hpp)

| Filter | Mô tả | Ứng dụng |
|--------|-------|----------|
| `LowPassFilter2p` | Butterworth 2nd-order IIR | Gyro @ 100 Hz, Accel @ 50 Hz |
| `NotchFilter` | Loại bỏ tần số cụ thể | Motor vibration |
| `MedianFilter` | Loại bỏ impulse noise | Baro spike rejection |

---

## 8. Triển khai (Build & Flash)

### Bước 1: Cấu hình NuttX

```bash
cd nuttxspace/nuttx

# Load board config cho STM32H743 (ví dụ custom board)
./tools/configure.sh boards/arm/stm32h7/spresense/configs/nsh

# Hoặc nếu có board config riêng:
./tools/configure.sh <your_board>/configs/uav
```

### Bước 2: Kích hoạt UAV stack qua menuconfig

```bash
make menuconfig
```

Điều hướng tới:

```
Application Configuration
  └─> UAV Avionics Stack   [*]
        ├─> uORB Message Bus                [*]
        ├─> UAV Drivers
        │     ├─> ICM-42688P IMU Driver     [*]
        │     └─> BMM150 Magnetometer       [*]
        ├─> Sensors Application             [*]
        ├─> Estimator Application           [*]
        ├─> State Application               [*]
        ├─> Telemetry Application           [*]
        └─> Calibration Application         [*]
```

> **Lưu ý:** Nhớ enable các tính năng NuttX cần thiết:
> - `CONFIG_SCHED_FIFO=y` (Real-time scheduling)
> - `CONFIG_SPI=y`, `CONFIG_I2C=y`
> - `CONFIG_SERIAL=y` (Telemetry UART)

### Bước 3: Build

```bash
# Build toàn bộ
make -j$(nproc)

# Nếu chỉ muốn build UAV app:
make -C ../apps/uav -j$(nproc)
```

Output: `nuttx/nuttx.bin` (hoặc `.elf` / `.hex` tùy cấu hình)

### Bước 4: Flash lên STM32H743

**Dùng ST-Link + OpenOCD:**

```bash
openocd -f interface/stlink.cfg \
        -f target/stm32h7x.cfg \
        -c "program nuttx.bin 0x08000000 verify reset exit"
```

**Dùng st-flash:**

```bash
st-flash write nuttx.bin 0x08000000
```

**Dùng STM32CubeProgrammer (GUI):**

Mở `nuttx.hex`, chọn `Download`, kết nối ST-Link.

---

## 9. Chạy trên phần cứng

Sau khi flash, kết nối serial terminal (115200 baud):

```bash
minicom -D /dev/ttyUSB0 -b 115200
# hoặc
screen /dev/ttyUSB0 115200
```

### Thứ tự khởi động bắt buộc

```bash
nsh> sensors start        # 1. PHẢI chạy đầu tiên — cung cấp data cho các app khác
nsh> estimator start      # 2. Sau sensors (subscribe sensor_combined)
nsh> state start          # 3. Sau estimator (subscribe vehicle_attitude)
nsh> telemetry start      # 4. Cuối cùng (subscribe tất cả)
```

> ⚠️ **Quan trọng:** Không đảo ngược thứ tự. Nếu `estimator_app` khởi động trước `sensors_app`, nó sẽ không có data và có thể block.

### Kiểm tra trạng thái hệ thống

```bash
nsh> sensors status
# Output:
# [sensors] Loop: 12345, Deadline misses: 0
# [sensors] IMU[0]: 12345 samples, 0 errors
# [sensors] Jitter: min=124us, max=127us, avg=125.1us
# [sensors] Health: NOMINAL

nsh> sensors health
# [health] MASTER_TICK: OK (beat 12s ago)
# [health] IMU_0: OK   IMU_1: OK   IMU_2: OK   IMU_3: OK
# [health] BARO: NOT_PRESENT
# [health] MAG:  OK
# [health] GPS:  NOT_PRESENT
# [health] EKF:  OK

nsh> sensors fusion
# [fusion] Mode: VOTING
# [fusion] Active IMUs: 4/4  (mask: 0x0F)
# [fusion] Outlier rejects: 0
```

---

## 10. Calibration cảm biến

> **Yêu cầu:** `sensors_app` phải đang chạy trước khi dùng `calib_app`.

### 10.1 Calibrate Gyroscope (Gyro Bias)

```bash
nsh> calib gyro
# Đặt board YÊN trên mặt phẳng phẳng
# Thu thập 5000 mẫu (~5 giây)
# → Tính bias trung bình trên 3 trục
# → Áp dụng ngay vào ICM42688P driver
```

**Thuật toán:** Average-at-rest
```
bias_x = mean(gyro_x[0..N])    [rad/s]
bias_y = mean(gyro_y[0..N])    [rad/s]
bias_z = mean(gyro_z[0..N])    [rad/s]
```

**Validation:** `|bias| < 0.5 rad/s` (~28.6 °/s). Nếu lớn hơn → board đang rung.

---

### 10.2 Calibrate Accelerometer (Accel Bias)

```bash
nsh> calib accel
# Đặt board NẰM NGANG trên mặt phẳng (Z+ hướng lên)
# Thu thập 1000 mẫu
# → Tính bias so với giá trị lý tưởng (X=0, Y=0, Z=-9.81 m/s²)
```

**Thuật toán:** Single-position gravity calibration (simplified):
```
bias_x = mean(accel_x) - 0.0
bias_y = mean(accel_y) - 0.0
bias_z = mean(accel_z) - (-9.80665)
```

**Validation:** `|accel| ≈ 9.81 m/s²`. Sai số > 2 m/s² → reject.

> **Ghi chú:** Để chính xác cao hơn, cần full 6-position calibration (chưa implement).

---

### 10.3 Calibrate Magnetometer (Hard-Iron)

```bash
nsh> calib mag
# Xoay board CHẬM theo tất cả hướng — 360° trên 3 mặt phẳng (pitch, roll, yaw)
# Thu thập 2000 mẫu (~100 giây ở 20 Hz)
# → Tính hard-iron offset bằng min-max sphere fit
```

**Thuật toán:** Min-max sphere fit:
```
offset_x = (max_x + min_x) / 2    [µT]
offset_y = (max_y + min_y) / 2    [µT]
offset_z = (max_z + min_z) / 2    [µT]
```

**Validation:**
- Cường độ từ trường ước lượng: nên trong khoảng 25–65 µT
- Tỷ lệ range giữa 3 trục nên > 0.5 (nếu không → xoay chưa đủ)

---

### 10.4 Xem và Reset calibration

```bash
nsh> calib status
# --- IMU (ICM42688P) ---
# Gyro bias:   X=+0.001234  Y=-0.000567  Z=+0.000891  rad/s
# Accel bias:  X=+0.0234    Y=-0.0123    Z=+0.0456    m/s²
# Accel scale: 1.000234
# Gyro  calibrated: YES (3 updates)
# Accel calibrated: YES (1 update)
#
# --- MAG (BMM150) ---
# Hard-iron: X=+12.3  Y=-8.7  Z=+2.1  µT

nsh> calib reset        # Reset tất cả về 0
```

---

## 11. Cấu hình Kconfig

Các tham số có thể chỉnh qua `make menuconfig` hoặc `.config`:

```kconfig
# === Master Timebase ===
CONFIG_UAV_MASTER_TICK_HZ=8000      # Tần số master tick (Hz)
CONFIG_UAV_MEDIUM_DIVIDER=8         # MEDIUM = 8kHz / 8 = 1kHz
CONFIG_UAV_SLOW_DIVIDER=80          # SLOW   = 8kHz / 80 = 100Hz

# === IMU ===
CONFIG_UAV_NUM_IMUS=4               # Số lượng IMU
CONFIG_UAV_IMU_RATE_HZ=1000        # Tần số đọc IMU (Hz)
CONFIG_UAV_SENSORS_PRIORITY=250     # Task priority (SCHED_FIFO)
CONFIG_UAV_SENSORS_STACKSIZE=8192   # Stack size (bytes)

# === DSP Filters ===
CONFIG_UAV_GYRO_CUTOFF_HZ=100       # Lowpass cutoff cho gyro
CONFIG_UAV_ACCEL_CUTOFF_HZ=50       # Lowpass cutoff cho accel

# === IMU Fusion ===
CONFIG_UAV_IMU_FUSION_MODE=0        # 0=VOTING, 1=WEIGHTED, 2=PRIMARY

# === Health Monitoring ===
CONFIG_UAV_HEARTBEAT_TIMEOUT_MS=50  # Timeout coi là missed beat
CONFIG_UAV_MISSED_BEATS_WARN=3      # Số miss → WARNING
CONFIG_UAV_MISSED_BEATS_FAIL=10     # Số miss → FAILSAFE

# === Calibration ===
CONFIG_UAV_CALIB_GYRO_SAMPLES=5000  # Số mẫu gyro calibration
CONFIG_UAV_CALIB_ACCEL_SAMPLES=1000 # Số mẫu accel calibration
CONFIG_UAV_CALIB_MAG_SAMPLES=2000   # Số mẫu mag calibration
```

---

## 12. Ràng buộc kỹ thuật

| Ràng buộc | Lý do |
|-----------|-------|
| **Không malloc/new trong real-time loop** | Tránh heap fragmentation, đảm bảo timing |
| **Dùng `SCHED_FIFO`** cho sensors, estimator | Đảm bảo preemption đúng priority |
| **Dùng `hrt_absolute_time()`** thay `clock_gettime()` | Overhead thấp hơn, chính xác hơn |
| **SPI1 chỉ thuộc `sensors_app`** | Single ownership — tránh race condition |
| **I2C1 chỉ thuộc `sensors_app`** | Như trên |
| **Giao tiếp qua uORB, không gọi hàm trực tiếp** | Fault isolation |
| **Tất cả buffer phải khai báo static** | Zero runtime allocation |

---

## 13. Roadmap

### Phase 1 — Hiện tại ✅
- [x] Master timebase (8 kHz, POSIX timer)
- [x] Lock-free ring buffer (SPSC)
- [x] DSP filters (Butterworth 2p, Notch, Median)
- [x] IMU fusion (Voting / Weighted / Primary)
- [x] Health monitoring (4 levels, per-component)
- [x] ICM-42688-P driver (SPI, calibration, self-test)
- [x] BMM150 driver (I2C, hard-iron calibration)
- [x] Interactive calibration app (Gyro / Accel / Mag)
- [x] uORB pub/sub messagin bus
- [x] sensors_app, estimator_app, state_app, telemetry_app

### Phase 2 — DMA & Hardware timestamps ⏳
- [ ] DMA SPI transfers (giảm CPU load)
- [ ] Hardware timestamps từ SPI FIFO
- [ ] Coning & sculling compensation

### Phase 3 — Full Navigation ❌
- [ ] Full EKF2 integration (position + vel)
- [ ] Baro driver (MS5611)
- [ ] GPS driver & NMEA/UBX parser
- [ ] Multi-sensor EKF fusion (Baro, Mag, GPS)
- [ ] Telemetry MAVLink output

---

## 14. Tham khảo

- [Apache NuttX Documentation](https://nuttx.apache.org/docs/latest/)
- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) — kiến trúc multi-app, uORB
- [VectorNav VN-100](https://www.vectornav.com/products/detail/vn-100) — IMU pipeline reference
- [STM32H743 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00314099.pdf)
- [ICM-42688-P Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)
- [BMM150 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/magnetometers/bmm150/)

---

*Tác giả: Trần Tiến Dũng — 2026*
