# lib/platform - Platform Abstraction Layer

## 📋 Tổng Quan

Module `platform` cung cấp **hardware abstraction** cho các tính năng liên quan đến thời gian và phần cứng cụ thể của NuttX/STM32.

### Đặc Điểm

- **Portable API:** Có thể port sang platform khác
- **High precision:** Microsecond resolution
- **Deterministic:** Cho realtime systems

---

## 📁 Files

```
platform/
├── hrt.h           # High-resolution timer API
├── hrt.c           # HRT implementation
├── timebase.h      # Master timebase API
├── timebase.c      # Multi-rate scheduler
└── spi_config.h    # SPI pin/bus configuration
```

---

## ⏱️ hrt.h - High Resolution Timer

### Mục Đích

Cung cấp thời gian chính xác ở độ phân giải **microsecond**. Đây là time source chuẩn cho toàn bộ hệ thống UAV.

### API

```c
#include <uav/lib/platform/hrt.h>

// Lấy timestamp hiện tại (microseconds từ boot)
uint64_t now = hrt_absolute_time();

// Lấy timestamp (milliseconds)
uint64_t now_ms = hrt_absolute_time_ms();

// Sleep chính xác
hrt_usleep(1000);  // 1 ms
```

### Implementation

Dựa trên `clock_systime_timespec()` của NuttX:

```c
uint64_t hrt_absolute_time(void)
{
    struct timespec ts;
    clock_systime_timespec(&ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + 
           (uint64_t)ts.tv_nsec / 1000ULL;
}
```

### Sử Dụng

```c
// Đo thời gian thực thi
uint64_t start = hrt_absolute_time();
do_something();
uint64_t elapsed = hrt_absolute_time() - start;
syslog(LOG_INFO, "Elapsed: %llu us\n", elapsed);

// Timestamp trong message
sensor_imu.timestamp = hrt_absolute_time();
orb_publish(ORB_ID(sensor_imu), pub, &sensor_imu);
```

---

## ⏲️ timebase.h - Master Timebase

### Mục Đích

Cung cấp **multi-rate scheduling** cho hệ thống Time-Triggered:

- **Master tick:** 8 kHz từ hardware timer
- **Domain subdivision:** FAST/MEDIUM/SLOW/ASYNC
- **Synchronization:** Tasks đồng bộ với domains

### Domains

| Domain | Rate | Divider | Use Case |
|--------|------|---------|----------|
| FAST | 8 kHz | 1 | IMU DMA, gyro integration |
| MEDIUM | 1 kHz | 8 | EKF predict, sensor fusion |
| SLOW | 100 Hz | 80 | Baro, Mag, GPS |
| ASYNC | Event | - | Telemetry, logging |

### API

```c
#include <uav/lib/platform/timebase.h>

// Khởi tạo (gọi một lần khi boot)
int ret = timebase_init();

// Task đợi tick của domain
while (!should_exit) {
    // Block cho đến khi có tick
    timebase_wait_domain(DOMAIN_MEDIUM);  // 1 kHz
    
    // Do work here...
}

// Lấy timestamp (microseconds)
uint64_t ts = timebase_get_timestamp();

// Cleanup
timebase_deinit();
```

### Sơ Đồ Hoạt Động

```
Hardware Timer (TIM5) @ 8 kHz
         │
         ▼ ISR mỗi 125 µs
┌─────────────────────────────────────────────────────────────────┐
│                        Timer ISR                                 │
│                                                                  │
│  tick_counter++;                                                 │
│                                                                  │
│  // FAST domain - mỗi tick                                      │
│  sem_post(&fast_sem);                                           │
│                                                                  │
│  if (tick_counter % 8 == 0) {                                   │
│      // MEDIUM domain - mỗi 8 ticks                             │
│      sem_post(&medium_sem);                                     │
│  }                                                               │
│                                                                  │
│  if (tick_counter % 80 == 0) {                                  │
│      // SLOW domain - mỗi 80 ticks                              │
│      sem_post(&slow_sem);                                       │
│  }                                                               │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────┐
│                         Tasks                                    │
│                                                                  │
│  sensors_task:                                                   │
│    while (1) {                                                  │
│        timebase_wait_domain(DOMAIN_MEDIUM);  // Block           │
│        read_imu();                                              │
│        publish_sensor_data();                                   │
│    }                                                            │
│                                                                  │
│  estimator_task:                                                │
│    while (1) {                                                  │
│        timebase_wait_domain(DOMAIN_MEDIUM);  // Block           │
│        run_ekf_predict();                                       │
│    }                                                            │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Jitter Statistics

```c
// Lấy thống kê jitter
timebase_jitter_stats_t stats;
timebase_get_jitter_stats(&stats);

printf("Max jitter: %lu us\n", stats.max_jitter_us);
printf("Avg jitter: %lu us\n", stats.avg_jitter_us);
printf("Miss count: %lu\n", stats.missed_ticks);
```

---

## 🔧 spi_config.h - SPI Configuration

### Mục Đích

Định nghĩa **pin assignments** và **bus configuration** cho SPI devices.

### Nội Dung

```c
// SPI bus cho IMUs
#define IMU_SPI_BUS         1
#define IMU_SPI_FREQ        10000000  // 10 MHz

// Chip select pins
#define IMU0_CS_GPIO        GPIO_SPI1_CS0
#define IMU1_CS_GPIO        GPIO_SPI1_CS1
#define IMU2_CS_GPIO        GPIO_SPI1_CS2
#define IMU3_CS_GPIO        GPIO_SPI1_CS3

// DMA configuration
#define IMU_DMA_STREAM_TX   DMA2_Stream3
#define IMU_DMA_STREAM_RX   DMA2_Stream2
```

---

## 📊 Performance Considerations

### 1. HRT Overhead

```c
// hrt_absolute_time() rất nhẹ (~100 ns)
// Có thể gọi nhiều lần trong loop
```

### 2. Timebase Semaphore

```c
// sem_wait() có overhead ~1-5 µs
// Không dùng trong code timing-critical (như ISR)
```

### 3. Jitter Target

```
Target jitter < 50 µs cho MEDIUM domain
Nếu jitter > 100 µs, có thể có task priority issue
```

---

## 🔗 Liên Kết

- **Usage:** `/apps/uav/sensors_app/sensors_main.cpp`
- **Timer config:** Board-specific files
- **Architecture:** `/apps/uav/ARCHITECTURE.md`
