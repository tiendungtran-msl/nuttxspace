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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: read_icm_
 *
 * Description:
 *   Đọc 1 byte từ thanh ghi của SPI slave device
 *
 * Input Parameters:
 *   dev      - Con trỏ đến cấu trúc spi_dev_t
 *   reg_addr - Địa chỉ thanh ghi
 *   value    - Con trỏ để lưu giá trị đọc được
 *
 * Returned Value:
 *   SPI_OK on success; Mã lỗi âm on failure
 *
 ****************************************************************************/

int spi_read_reg(spi_dev_t *dev, uint8_t reg_addr, uint8_t *value);