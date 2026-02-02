# lib/drivers_framework - Driver Framework

## 📋 Tổng Quan

Module `drivers_framework` cung cấp **infrastructure chung** cho tất cả device drivers:
- **DeviceId:** Định danh duy nhất cho mỗi device
- **BusType:** Enum cho loại bus (SPI, I2C, UART...)
- **DeviceType:** Constants cho các loại sensor

---

## 📁 Files

```
drivers_framework/
├── Makefile
└── device_id.hpp    # Device identification structures
```

---

## 🆔 DeviceId

### Mục Đích

Mỗi sensor trong hệ thống cần một **ID duy nhất** để:
- Phân biệt các sensor cùng loại (4 IMU ICM42688P)
- Liên kết calibration data với đúng sensor
- Debug và logging

### Cấu Trúc (32-bit Union)

```cpp
union DeviceId {
    struct {
        uint8_t devtype;    // Loại device (ICM42688P, BMP280, ...)
        uint8_t address;    // I2C address hoặc SPI CS index
        uint8_t bus;        // Bus number (SPI1, SPI2, ...)
        uint8_t bus_type;   // BusType enum
    } fields;

    uint32_t value;         // Truy cập toàn bộ 32-bit
};
```

### Layout

```
 31          24 23          16 15           8 7            0
┌─────────────┬──────────────┬──────────────┬──────────────┐
│  bus_type   │     bus      │   address    │   devtype    │
│   (SPI=2)   │    (SPI4)    │    (CS=0)    │ (ICM42688P)  │
└─────────────┴──────────────┴──────────────┴──────────────┘
```

### Ví Dụ

```cpp
#include <uav/lib/drivers_framework/device_id.hpp>

using namespace drivers;

// Tạo DeviceId cho IMU 0 trên SPI4, CS 0
DeviceId imu0_id;
imu0_id.fields.devtype = DeviceType::ICM42688P;
imu0_id.fields.address = 0;  // CS index
imu0_id.fields.bus = 4;      // SPI4
imu0_id.fields.bus_type = static_cast<uint8_t>(BusType::SPI);

// Print dạng hex
printf("IMU0 DeviceId: 0x%08X\n", imu0_id.value);

// Check validity
if (imu0_id.is_valid()) {
    // Use it
}

// Compare
DeviceId imu1_id;
imu1_id.fields = {DeviceType::ICM42688P, 1, 4, static_cast<uint8_t>(BusType::SPI)};

if (imu0_id != imu1_id) {
    // Khác nhau vì address (CS) khác
}
```

---

## 🔌 BusType Enum

### Định Nghĩa

```cpp
enum class BusType : uint8_t {
    UNKNOWN = 0,
    I2C     = 1,
    SPI     = 2,
    UART    = 3,
    UAVCAN  = 4,
    SDMMC   = 5
};
```

### Sử Dụng

```cpp
BusType bus = BusType::SPI;

switch (bus) {
    case BusType::SPI:
        init_spi_driver();
        break;
    case BusType::I2C:
        init_i2c_driver();
        break;
    default:
        break;
}
```

---

## 📊 DeviceType Constants

### IMU Sensors

| Constant | Value | Chip |
|----------|-------|------|
| `MPU6050` | 0x09 | InvenSense MPU-6050 |
| `MPU9250` | 0x0A | InvenSense MPU-9250 |
| `ICM42688P` | 0x0B | TDK ICM-42688-P |
| `BMI088_ACCEL` | 0x0D | Bosch BMI088 (accel) |
| `BMI088_GYRO` | 0x0E | Bosch BMI088 (gyro) |

### Barometer

| Constant | Value | Chip |
|----------|-------|------|
| `BMP280` | 0x20 | Bosch BMP280 |
| `MS5611` | 0x21 | TE MS5611 |

### Magnetometer

| Constant | Value | Chip |
|----------|-------|------|
| `HMC5883L` | 0x30 | Honeywell HMC5883L |
| `QMC5883L` | 0x31 | QST QMC5883L |

### Usage

```cpp
// Trong driver ICM42688P
DeviceId id;
id.fields.devtype = DeviceType::ICM42688P;

// Trong calibration system
if (id.fields.devtype == DeviceType::ICM42688P) {
    load_icm42688p_calibration(id);
}
```

---

## 💡 Use Cases

### 1. Sensor Registration

```cpp
// Khi driver init, tạo DeviceId và register
void icm42688p_driver_init(int spi_bus, int cs_index)
{
    DeviceId id;
    id.fields.devtype = DeviceType::ICM42688P;
    id.fields.address = cs_index;
    id.fields.bus = spi_bus;
    id.fields.bus_type = static_cast<uint8_t>(BusType::SPI);
    
    // Register với sensor subsystem
    sensor_register(id, &icm42688p_ops);
}
```

### 2. Calibration Lookup

```cpp
// Load calibration cho sensor cụ thể
bool load_calibration(DeviceId id, SensorCalibration* cal)
{
    // Use id.value as key
    char key[32];
    snprintf(key, sizeof(key), "CAL_%08X", id.value);
    
    return param_load(key, cal, sizeof(*cal));
}
```

### 3. Telemetry

```cpp
// Include device ID trong telemetry packet
struct SensorStatus {
    uint32_t device_id;   // DeviceId.value
    uint8_t health;
    uint8_t error_count;
};
```

---

## 📐 Design Rationale

### Tại Sao Dùng Union?

```cpp
// Option 1: Separate struct (khó compare)
struct DeviceId {
    uint8_t devtype;
    uint8_t address;
    uint8_t bus;
    uint8_t bus_type;
};
// Compare: phải so từng field

// Option 2: Union (dễ compare)
union DeviceId {
    struct {...} fields;
    uint32_t value;
};
// Compare: chỉ cần so value (1 operation)
```

### Tại Sao 32-bit?

- Đủ thông tin: 4 × 8-bit = 32-bit
- Align tốt: natural alignment trên 32-bit MCU
- So sánh nhanh: 1 instruction compare
- Hash đơn giản: dùng trực tiếp làm key

---

## 🔗 Liên Kết

- **Usage:** `/apps/uav/drivers/`
- **Calibration:** `/apps/uav/lib/calibration/`
- **IMU Driver:** `/apps/uav/drivers/imu/`
