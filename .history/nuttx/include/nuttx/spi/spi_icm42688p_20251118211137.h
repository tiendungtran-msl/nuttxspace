#ifndef __INCLUDE_NUTTX_SPI_SPI_ICM42688P_H
#define __INCLUDE_NUTTX_SPI_SPI_ICM42688P_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <nuttx/spi/spi_transfer.h>
#include <syslog.h>
#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 struct icm42688p_dev_s
 {
    spi_dev_s spi;               /* SPI device handle */
    bool initialized;            /* Initialization flag */
    uint8_t current_bank;        /* Current register bank */
    uint8_t gyro_fs;             /* Gyro full-scale setting */
    float gyro_sensitivity;      /* Gyro sensitivity (LSB/dps) */
    uint8_t accel_fs;            /* Accel full-scale setting */
    float accel_sensitivity;     /* Accel sensitivity (LSB/g) */
    uint32_t sample_count;       /* Sample count */
    uint32_t error_count;        /* Communication error count */
 };
/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: read_icm_reg
 *
 * Description:
 *   Đọc 1 byte từ thanh ghi của SPI slave device
 *
 * Input Parameters:
 *   dev      - Con trỏ đến cấu trúc spi_dev_s
 *   reg_addr - Địa chỉ thanh ghi
 *   value    - Con trỏ để lưu giá trị đọc được
 *
 * Returned Value:
 *   SPI_OK on success; Mã lỗi âm on failure
 *
 ****************************************************************************/

int spi_read_reg(spi_dev_t *dev, uint8_t reg_addr, uint8_t *value);