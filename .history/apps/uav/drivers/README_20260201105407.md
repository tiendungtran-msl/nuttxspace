# Drivers - Hardware Abstraction Layer

## Tổng quan

Thư mục này chứa các driver phần cứng cho UAV:
- IMU (ICM-42688P, MPU6050, ...)
- Magnetometer (RM3100, HMC5883, ...)
- Barometer (MS5611, BMP280, ...)
- GPS (u-blox, ...)

## Nguyên tắc thiết kế

### 1. Single Ownership

Mỗi bus (SPI/I2C) chỉ có một app sở hữu:
- SPI1: sensors_app sở hữu, đọc IMU
- I2C1: sensors_app sở hữu, đọc Mag/Baro

### 2. Layered Architecture

```
┌─────────────────────────────────────────┐
│            sensors_app                  │  Application
├─────────────────────────────────────────┤
│        Driver Interface                 │  API
│   (init, read, configure, ...)         │
├─────────────────────────────────────────┤
│         Device Driver                   │  Hardware specific
│   (ICM42688P, RM3100, MS5611)          │
├─────────────────────────────────────────┤
│         NuttX SPI/I2C                   │  Platform
└─────────────────────────────────────────┘
```

### 3. Driver Interface

Mỗi loại sensor có interface chuẩn:

```cpp
// IMU interface
struct imu_driver_s {
    int (*init)(const imu_config_s* config);
    int (*read)(imu_data_s* data);
    int (*configure)(const imu_config_s* config);
    int (*self_test)(void);
};

// Mag interface
struct mag_driver_s {
    int (*init)(const mag_config_s* config);
    int (*read)(mag_data_s* data);
    int (*calibrate)(mag_cal_s* cal);
};
```

## Cấu trúc thư mục

```
drivers/
├── README.md           # File này
├── Makefile            # Build all drivers
├── Kconfig             # Config menu
│
├── imu/                # IMU drivers
│   ├── imu_interface.hpp
│   ├── icm42688p/
│   │   ├── icm42688p.hpp
│   │   ├── icm42688p.cpp
│   │   └── icm42688p_registers.hpp
│   └── mpu6050/
│       └── ...
│
├── mag/                # Magnetometer drivers
│   ├── mag_interface.hpp
│   └── rm3100/
│       └── ...
│
├── baro/               # Barometer drivers
│   ├── baro_interface.hpp
│   └── ms5611/
│       └── ...
│
└── gps/                # GPS drivers
    ├── gps_interface.hpp
    └── ublox/
        └── ...
```

## Usage trong sensors_app

```cpp
// sensors_main.cpp

#include <uav/drivers/imu/icm42688p/icm42688p.hpp>

// Init
ICM42688P imu;
imu.init(spi_dev, cs_gpio);

// Read
imu_data_s data;
imu.read(&data);
```

## Calibration

Calibration data lưu trong filesystem:
- `/dev/calibration/imu0.cal`
- `/dev/calibration/mag0.cal`

Format: Binary struct với version number.

## Testing

```bash
# Test IMU driver
nsh> icm42688p test

# Check self-test
nsh> icm42688p selftest

# View raw data
nsh> icm42688p dump
```
