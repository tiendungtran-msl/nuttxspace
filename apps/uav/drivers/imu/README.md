# drivers/imu - IMU Drivers

## 📋 Tổng Quan

Thư mục `drivers/imu` chứa **low-level drivers** cho các sensor IMU (Inertial Measurement Unit):
- **ICM42688P:** TDK/InvenSense 6-axis IMU (gyro + accelerometer)
- Future: BMI088, MPU6500, etc.

---

## 📁 Files

```
imu/
├── Makefile
├── icm42688p.hpp     # Driver header + register definitions
├── icm42688p.cpp     # Driver implementation
└── icm42688p/        # Additional files (config, etc.)
```

---

## 🔧 ICM42688P Driver

### Chip Overview

| Feature | Specification |
|---------|---------------|
| Manufacturer | TDK InvenSense |
| Axes | 6-axis (3 gyro + 3 accel) |
| Interface | SPI up to 24 MHz |
| Gyro Range | ±2000 °/s (full scale) |
| Accel Range | ±16g (full scale) |
| Resolution | 16-bit |
| ODR | Up to 32 kHz |
| Temperature | On-chip sensor |

### Register Map

```cpp
// Key registers (Bank 0)
#define ICM42688P_REG_WHO_AM_I      0x75  // = 0x47
#define ICM42688P_REG_PWR_MGMT0     0x4E  // Power control
#define ICM42688P_REG_GYRO_CONFIG0  0x4F  // Gyro range & ODR
#define ICM42688P_REG_ACCEL_CONFIG0 0x50  // Accel range & ODR
#define ICM42688P_REG_TEMP_DATA1    0x1D  // Temperature (high byte)
#define ICM42688P_REG_ACCEL_DATA_X1 0x1F  // Accel X (high byte)
#define ICM42688P_REG_GYRO_DATA_X1  0x25  // Gyro X (high byte)
```

### Data Layout

```
Sensor data registers (14 bytes total):

Addr  Register        Description
────  ──────────────  ─────────────────
0x1D  TEMP_DATA1      Temperature [15:8]
0x1E  TEMP_DATA0      Temperature [7:0]
0x1F  ACCEL_DATA_X1   Accel X [15:8]
0x20  ACCEL_DATA_X0   Accel X [7:0]
0x21  ACCEL_DATA_Y1   Accel Y [15:8]
0x22  ACCEL_DATA_Y0   Accel Y [7:0]
0x23  ACCEL_DATA_Z1   Accel Z [15:8]
0x24  ACCEL_DATA_Z0   Accel Z [7:0]
0x25  GYRO_DATA_X1    Gyro X [15:8]
0x26  GYRO_DATA_X0    Gyro X [7:0]
0x27  GYRO_DATA_Y1    Gyro Y [15:8]
0x28  GYRO_DATA_Y0    Gyro Y [7:0]
0x29  GYRO_DATA_Z1    Gyro Z [15:8]
0x2A  GYRO_DATA_Z0    Gyro Z [7:0]
```

---

## 📊 Data Structures

### Raw Data (16-bit integers)

```cpp
struct icm42688p_raw_data_s {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};
```

### Scaled Data (floats)

```cpp
struct icm42688p_data_s {
    float accel_x;      // m/s²
    float accel_y;      // m/s²
    float accel_z;      // m/s²
    float gyro_x;       // rad/s
    float gyro_y;       // rad/s
    float gyro_z;       // rad/s
    float temperature;  // °C
};
```

---

## ⚙️ Scale Factors

### Gyroscope (±2000 °/s)

```cpp
// Raw → rad/s
#define ICM42688P_GYRO_SCALE_2000DPS  (2000.0f / 32768.0f * DEG_TO_RAD)
                                     // = 0.001065264 rad/s per LSB

// Example:
// Raw = 1000 → 1000 × 0.001065264 = 1.065 rad/s ≈ 61°/s
```

### Accelerometer (±16g)

```cpp
// Raw → m/s²
#define ICM42688P_ACCEL_SCALE_16G  (16.0f / 32768.0f * GRAVITY_MSS)
                                   // = 0.004788403 m/s² per LSB

// Example:
// Raw = 2048 → 2048 × 0.004788403 = 9.81 m/s² ≈ 1g
```

### Temperature

```cpp
// Raw → °C
#define ICM42688P_TEMP_SCALE   (1.0f / 132.48f)
#define ICM42688P_TEMP_OFFSET  25.0f

// Formula: temp_C = raw / 132.48 + 25
```

---

## 🔧 API

### Initialization

```cpp
#include <uav/drivers/imu/icm42688p.hpp>

// Cần SPI device từ NuttX
struct spi_dev_s *spi = nuttx_spi_initialize(4);  // SPI4

// Tạo driver instance
ICM42688P imu(spi, 0);  // Instance 0

// Initialize
int ret = imu.init();
if (ret != 0) {
    printf("ICM42688P init failed: %d\n", ret);
    return ret;
}

// Check status
if (!imu.is_ok()) {
    printf("ICM42688P not responding\n");
}
```

### Read Data

```cpp
// Read raw (16-bit integers)
icm42688p_raw_data_s raw;
ret = imu.read_raw(&raw);
if (ret == 0) {
    printf("Raw gyro X: %d\n", raw.gyro_x);
}

// Read scaled (floats in SI units)
icm42688p_data_s data;
ret = imu.read(&data);
if (ret == 0) {
    printf("Gyro X: %.3f rad/s\n", data.gyro_x);
    printf("Accel Z: %.2f m/s²\n", data.accel_z);
    printf("Temp: %.1f °C\n", data.temperature);
}
```

---

## 📡 SPI Communication

### Timing

```
SPI Clock: 8 MHz (conservative)
          24 MHz (maximum per datasheet)

Read transaction:
┌────────────────────────────────────────────────────────────┐
│ CS ────┐                                            ┌───── │
│        └────────────────────────────────────────────┘      │
│                                                            │
│ MOSI ──►[Reg | 0x80]                                       │
│                      ◄── Dummy ──►                         │
│ MISO ────────────────────────────►[Data]                   │
│                                                            │
│ Total: 16 bits = 2 µs @ 8 MHz                              │
└────────────────────────────────────────────────────────────┘
```

### Burst Read

```cpp
// Đọc tất cả sensor data (14 bytes) trong 1 transaction
// Tối ưu bandwidth và latency

int read_all_sensors(icm42688p_raw_data_s *data)
{
    uint8_t tx[15] = {ICM42688P_REG_TEMP_DATA1 | 0x80};  // Read flag
    uint8_t rx[15];
    
    // Single 15-byte transfer
    spi_transfer(m_spi, tx, rx, 15);
    
    // Parse data (bytes 1-14)
    data->temp    = (rx[1] << 8) | rx[2];
    data->accel_x = (rx[3] << 8) | rx[4];
    data->accel_y = (rx[5] << 8) | rx[6];
    data->accel_z = (rx[7] << 8) | rx[8];
    data->gyro_x  = (rx[9] << 8) | rx[10];
    data->gyro_y  = (rx[11] << 8) | rx[12];
    data->gyro_z  = (rx[13] << 8) | rx[14];
    
    return 0;
}
```

---

## 🔧 Configuration

### Power Modes

```cpp
// Low Noise mode (default - best accuracy)
write_reg(ICM42688P_REG_PWR_MGMT0,
          ICM42688P_GYRO_MODE_LN | ICM42688P_ACCEL_MODE_LN);

// Low Power mode (reduced current, lower accuracy)
write_reg(ICM42688P_REG_PWR_MGMT0,
          ICM42688P_GYRO_MODE_LP | ICM42688P_ACCEL_MODE_LP);
```

### Output Data Rate (ODR)

```cpp
// Gyro ODR options
ICM42688P_GYRO_ODR_32KHZ   // 32 kHz
ICM42688P_GYRO_ODR_16KHZ   // 16 kHz
ICM42688P_GYRO_ODR_8KHZ    // 8 kHz
ICM42688P_GYRO_ODR_4KHZ    // 4 kHz
ICM42688P_GYRO_ODR_2KHZ    // 2 kHz
ICM42688P_GYRO_ODR_1KHZ    // 1 kHz (default)
ICM42688P_GYRO_ODR_500HZ   // 500 Hz
ICM42688P_GYRO_ODR_200HZ   // 200 Hz
```

### Full Scale Range

```cpp
// Gyro range options
ICM42688P_GYRO_FS_2000DPS  // ±2000 °/s (default, most range)
ICM42688P_GYRO_FS_1000DPS  // ±1000 °/s
ICM42688P_GYRO_FS_500DPS   // ±500 °/s
ICM42688P_GYRO_FS_250DPS   // ±250 °/s (highest resolution)

// Accel range options
ICM42688P_ACCEL_FS_16G     // ±16g (default, most range)
ICM42688P_ACCEL_FS_8G      // ±8g
ICM42688P_ACCEL_FS_4G      // ±4g
ICM42688P_ACCEL_FS_2G      // ±2g (highest resolution)
```

---

## 💡 Best Practices

### 1. WHO_AM_I Check

```cpp
// Always verify chip identity at init
int verify_whoami()
{
    uint8_t who = read_reg(ICM42688P_REG_WHO_AM_I);
    if (who != ICM42688P_WHO_AM_I_VALUE) {  // 0x47
        UAV_ERROR("ICM42688P WHO_AM_I mismatch: got 0x%02X, expected 0x47", who);
        return -ENODEV;
    }
    return 0;
}
```

### 2. Soft Reset

```cpp
// Reset chip to known state
void soft_reset()
{
    write_reg(ICM42688P_REG_DEVICE_CONFIG, 0x01);  // Soft reset bit
    usleep(1000);  // Wait 1ms for reset complete
}
```

### 3. Error Handling

```cpp
// Check for stuck sensor
int read_with_validation(icm42688p_data_s *data)
{
    int ret = read(data);
    if (ret != 0) return ret;
    
    // Check for obviously invalid data
    if (!std::isfinite(data->gyro_x) ||
        fabsf(data->accel_z) < 0.1f) {  // Should be ~9.81 when still
        return -EIO;
    }
    
    return 0;
}
```

---

## 📊 Performance

| Metric | Value |
|--------|-------|
| read() time | ~20 µs @ 8 MHz SPI |
| read_raw() time | ~18 µs |
| Init time | ~5 ms |
| Memory footprint | ~100 bytes per instance |

---

## 🔗 Liên Kết

- **Usage:** `/apps/uav/sensors_app/`
- **SPI Base:** `/apps/uav/drivers/spi/`
- **Calibration:** `/apps/uav/lib/calibration/`
- **Datasheet:** [ICM-42688-P Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)
