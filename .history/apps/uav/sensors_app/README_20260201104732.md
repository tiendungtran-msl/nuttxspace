# Sensors Application

## Tổng Quan

`sensors_app` là app có **priority cao nhất** trong hệ thống, chịu trách nhiệm
thu thập dữ liệu từ tất cả cảm biến và publish lên uORB.

## Vai Trò

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           SENSORS_APP                                       │
│                       Priority: 250 (highest)                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐   ┌─────────────┐     │
│  │ ICM42688P   │   │   BMM150    │   │   MS5611    │   │   u-blox    │     │
│  │  (IMU x4)   │   │   (Mag)     │   │   (Baro)    │   │   (GPS)     │     │
│  └──────┬──────┘   └──────┬──────┘   └──────┬──────┘   └──────┬──────┘     │
│         │                 │                 │                 │            │
│         ▼                 ▼                 ▼                 ▼            │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │                      Main Loop @ 1kHz                              │    │
│  │  - Poll IMU mỗi 1ms                                                │    │
│  │  - Poll Mag mỗi 10ms                                               │    │
│  │  - Poll Baro mỗi 20ms                                              │    │
│  │  - Poll GPS khi có data (interrupt/UART)                           │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│         │                 │                 │                 │            │
│         ▼                 ▼                 ▼                 ▼            │
└─────────────────────────────────────────────────────────────────────────────┘
         │                 │                 │                 │
         ▼                 ▼                 ▼                 ▼
    sensor_imu[4]     sensor_mag       sensor_baro       sensor_gps
      (uORB)           (uORB)           (uORB)            (uORB)
```

## Tại Sao Priority Cao Nhất?

1. **Timing Critical**: IMU cần sample đều @ 1kHz, jitter > 50µs gây drift
2. **Data Freshness**: Estimator cần data mới nhất
3. **Hardware Ownership**: Tránh race condition trên SPI/I2C
4. **Determinism**: Không bị preempt bởi estimator hay logger

## Thiết Kế

### Single Owner Principle

Chỉ `sensors_app` được quyền truy cập hardware:
- SPI1: Tất cả IMU
- I2C1: Mag, Baro
- UART: GPS

Không app nào khác được gọi trực tiếp vào driver.

### Rate-Based Polling

```
1000 Hz │ IMU ─────────────────────────────────────────────────────────►
        │
 100 Hz │     Mag ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄►
        │
  50 Hz │         Baro ············································►
        │
   5 Hz │               GPS ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○►
        └──────────────────────────────────────────────────────────► time
```

### Static Memory

- Tất cả buffer phải static (không malloc trong loop)
- Message struct được reuse
- Driver instances là global objects

## File Structure

```
sensors_app/
├── README.md               # File này
├── Makefile
├── Kconfig
├── sensors_main.cpp        # Entry point + main loop
├── imu_publisher.hpp       # IMU polling logic
├── imu_publisher.cpp
├── (mag_publisher.*)       # Tương lai
├── (baro_publisher.*)      # Tương lai
└── (gps_publisher.*)       # Tương lai
```

## Khởi Động

```bash
nsh> sensors start        # Start sensors task
[sensors] Starting...
[sensors] Found 4 IMUs
[sensors] Running @ 1000 Hz

nsh> sensors status       # Xem trạng thái
[sensors] IMU[0]: 1000 samples/s
[sensors] IMU[1]: 1000 samples/s
[sensors] IMU[2]: 1000 samples/s
[sensors] IMU[3]: 1000 samples/s

nsh> sensors stop         # Dừng
[sensors] Stopped
```

## Ràng Buộc

1. **Không blocking call**: Tất cả I/O phải non-blocking hoặc có timeout ngắn
2. **Không printf trong loop**: Dùng syslog nếu cần debug
3. **Timestamp từ HRT**: `hrt_absolute_time()` không phải `clock_gettime()`
4. **Deadline miss detection**: Nếu miss > 2 cycles liên tiếp, log warning
