/****************************************************************************
 * apps/uav/lib/platform/spi_config.h
 *
 * Cấu hình SPI cho board UAV
 * Định nghĩa bus và device IDs cho các cảm biến
 ****************************************************************************/

#pragma once

#include <nuttx/config.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

/* SPI bus dùng cho IMU */
#define NUTTX_SPI_BUS_IMU  1

/**
 * SPI device IDs (chip select)
 * Phải khớp với cấu hình GPIO của board
 */
enum {
    SPIDEV_IMU      = 1,    /* ICM42688P sensor 0 (CS_IMU0) */
    SPIDEV_IMU1     = 2,    /* ICM42688P sensor 1 (CS_IMU1) */
    SPIDEV_IMU2     = 3,    /* ICM42688P sensor 2 (CS_IMU2) */
    SPIDEV_IMU3     = 4,    /* ICM42688P sensor 3 (CS_IMU3) */
    SPIDEV_MAG      = 5,    /* Magnetometer (nếu dùng SPI) */
    SPIDEV_BARO     = 6,    /* Barometer */
    SPIDEV_FLASH    = 7,    /* External flash */
};

/* SPI frequency limits (Hz) */
#define SPI_FREQ_SLOW   1000000     /* 1 MHz - cho initialization */
#define SPI_FREQ_FAST   10000000    /* 10 MHz - cho data transfer */
#define SPI_FREQ_MAX    20000000    /* 20 MHz - maximum safe speed */

/**
 * @brief Kiểm tra board có SPI bus này không
 * @param bus Số bus SPI (1, 2, 3...)
 * @return true nếu bus tồn tại
 */
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

    /* Fallback: kiểm tra runtime bằng cách mở device */
    char path[16];
    snprintf(path, sizeof(path), "/dev/spi%d", bus);

    int fd = open(path, O_RDONLY);
    if (fd >= 0) {
        close(fd);
        return true;
    }

    return false;
}
