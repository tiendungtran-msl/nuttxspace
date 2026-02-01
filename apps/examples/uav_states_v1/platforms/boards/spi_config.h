#pragma once

#include <nuttx/config.h>

#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

// SPI bus definitions
#define NUTTX_SPI_BUS_IMU  1

// SPI device IDs (chip select)
// These should match your board's GPIO configuration
enum {
    SPIDEV_IMU      = 1,    // ICM42688P sensor 0
    SPIDEV_IMU1     = 2,    // ICM42688P sensor 1
    SPIDEV_IMU2     = 3,    // ICM42688P sensor 2
    SPIDEV_IMU3     = 4,    // ICM42688P sensor 3
    SPIDEV_MAG      = 5,    // HMC5883L (if SPI version)
    SPIDEV_BARO     = 6,    // BMP280
    SPIDEV_FLASH    = 7,    // External flash
};

// SPI frequency limits (Hz)
#define SPI_FREQ_SLOW   1000000     // 1 MHz - for initialization
#define SPI_FREQ_FAST   10000000    // 10 MHz - for data transfer
#define SPI_FREQ_MAX    20000000    // 20 MHz - maximum safe speed

// Check if board has the bus
static inline bool board_has_spi_bus(int bus)
{
#if defined(CONFIG_STM32H7_SPI1) || defined(CONFIG_STM32_SPI1)
    if (bus == 1) return true;
#endif
#if defined(CONFIG_STM32H7_SPI2) || defined(CONFIG_STM32_SPI2)
    if (bus == 2) return true;
#endif
#if defined(CONFIG_STM32H7_SPI3) || defined(CONFIG_STM32_SPI3)
    if (bus == 3) return true;
#endif

    /* Fallback: match runtime reality. If the SPI character device exists,
     * the board has that SPI bus regardless of which Kconfig symbol name is used.
     */
    char path[16];
    snprintf(path, sizeof(path), "/dev/spi%d", bus);

    int fd = open(path, O_RDONLY);
    if (fd >= 0) {
        close(fd);
        return true;
    }

    return false;
}