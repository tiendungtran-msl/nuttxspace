#pragma once

#include <stdint.h>

namespace drivers {

/** Device bus type */
enum class BusType :  uint8_t {
    UNKNOWN = 0,
    I2C     = 1,
    SPI     = 2,
    UART    = 3,
    UAVCAN  = 4
};

/** Device ID structure (32-bit) */
union DeviceId {
    struct {
        uint8_t devtype;      // Device type (sensor specific)
        uint8_t address;      // I2C address or SPI CS index
        uint8_t bus;          // Bus number
        uint8_t bus_type;     // BusType enum
    } fields;
    
    uint32_t value;
    
    DeviceId() : value(0) {}
    explicit DeviceId(uint32_t val) : value(val) {}
    
    bool is_valid() const { return value != 0; }
    
    bool operator==(const DeviceId& other) const {
        return value == other.value;
    }
    
    bool operator!=(const DeviceId& other) const {
        return value != other. value;
    }
};

/** Device types */
namespace DeviceType {
    constexpr uint8_t UNKNOWN       = 0x00;
    
    // IMU
    constexpr uint8_t MPU6050       = 0x09;
    constexpr uint8_t MPU9250       = 0x0A;
    constexpr uint8_t ICM42688P     = 0x0B;
    constexpr uint8_t BM150         = 0x0C;
    constexpr uint8_t BMI088_ACCEL  = 0x0D;
    constexpr uint8_t BMI088_GYRO   = 0x0E;
    
    // Barometer
    constexpr uint8_t BMP280        = 0x20;
    constexpr uint8_t MS5611        = 0x21;
    
    // Magnetometer
    constexpr uint8_t HMC5883L      = 0x30;
    constexpr uint8_t QMC5883L      = 0x31;
}

} // namespace drivers