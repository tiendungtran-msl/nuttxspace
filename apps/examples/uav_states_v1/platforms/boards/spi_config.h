#pragma once

#include <nuttx/config.h>

// SPI bus definitions
#define NUTTX_SPI_BUS_IMU  1

// SPI device IDs (chip select)
// These should match your board's GPIO configuration
enum {
    SPIDEV_IMU      = 1,    // MPU6050/ICM42688P
    SPIDEV_MAG      = 2,    // HMC5883L (if SPI version)
    SPIDEV_BARO     = 3,    // BMP280
    SPIDEV_FLASH    = 4,    // External flash
};

// SPI frequency limits (Hz)
#define SPI_FREQ_SLOW   1000000     // 1 MHz - for initialization
#define SPI_FREQ_FAST   10000000    // 10 MHz - for data transfer
#define SPI_FREQ_MAX    20000000    // 20 MHz - maximum safe speed

// Check if board has the bus
static inline bool board_has_spi_bus(int bus)
{
#ifdef CONFIG_STM32_SPI1
    if (bus == 1) return true;
#endif
#ifdef CONFIG_STM32_SPI2
    if (bus == 2) return true;
#endif
#ifdef CONFIG_STM32_SPI3
    if (bus == 3) return true;
#endif
    return false;
}