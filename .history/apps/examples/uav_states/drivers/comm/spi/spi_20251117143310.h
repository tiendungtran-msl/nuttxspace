/****************************************************************************
 * apps/examples/uav_states/drivers/comm/spi/spi.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_UAV_STATES_DRIVERS_COMM_SPI_SPI_H
#define __APPS_EXAMPLES_UAV_STATES_DRIVERS_COMM_SPI_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI Return Codes */

#define SPI_OK              0
#define SPI_ERR_INIT       -1
#define SPI_ERR_OPEN       -2
#define SPI_ERR_IOCTL      -3
#define SPI_ERR_WRITE      -4
#define SPI_ERR_READ       -5
#define SPI_ERR_TRANSFER   -6
#define SPI_ERR_INVALID    -7

/* SPI Read/Write Bits */

#define SPI_READ_BIT        0x80
#define SPI_WRITE_BIT       0x00

/* Default SPI Configuration */

#define SPI_DEFAULT_FREQUENCY  1000000  /* 1MHz */
#define SPI_DEFAULT_MODE       0        /* CPOL=0, CPHA=0 */
#define SPI_DEFAULT_NBITS      8
#define SPI_MAX_RETRIES        3

/* CS Delay (microseconds) */

#define SPI_CS_DELAY_US        1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Thuộc tính của một thiết bị kết nối SPI (8) */

typedef struct spi_device
{
  int fd;                    /* File descriptor */
  uint8_t bus;               /* SPI bus number (0, 1, 2, ...) */
  uint8_t devid;             /* Device ID / CS line */
  uint32_t frequency;        /* SPI clock frequency (Hz) */
  uint8_t mode;              /* SPI mode (0-3) */
  uint8_t nbits;             /* Bits per word (usually 8) */
  char devpath[32];          /* Device path (e.g., "/dev/spi1") */
  bool initialized;          /* Initialization status */
  
  /* CS control function pointer (optional, can use board-specific) */
  void (*cs_select)(uint8_t devid, bool selected);
} spi_dev_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: spi_init
 *
 * Description:
 *   Khởi tạo SPI device từ user space
 *
 * Input Parameters:
 *   dev       - Con trỏ đến cấu trúc spi_dev_t
 *   bus       - Số bus SPI (0, 1, 2, ...)
 *   devid     - Device ID / CS line
 *   frequency - Tần số SPI (Hz), 0 = sử dụng mặc định
 *   mode      - SPI mode (0-3), -1 = sử dụng mặc định
 *
 * Returned Value:
 *   SPI_OK on success; Mã lỗi âm on failure
 *
 ****************************************************************************/

int spi_init(spi_dev_t *dev, uint8_t bus, uint8_t devid, 
             uint32_t frequency, int8_t mode);

/****************************************************************************
 * Name: spi_deinit
 *
 * Description:
 *   Hủy khởi tạo và đóng SPI device
 *
 * Input Parameters:
 *   dev - Con trỏ đến cấu trúc spi_dev_t
 *
 * Returned Value:
 *   SPI_OK on success; Mã lỗi âm on failure
 *
 ****************************************************************************/

int spi_deinit(spi_dev_t *dev);

/****************************************************************************
 * Name: spi_set_cs_callback
 *
 * Description:
 *   Đăng ký hàm callback để điều khiển CS (chip select)
 *   Nếu không đặt, sẽ sử dụng board-specific function
 *
 * Input Parameters:
 *   dev       - Con trỏ đến cấu trúc spi_dev_t
 *   cs_select - Con trỏ hàm callback (devid, selected)
 *
 ****************************************************************************/

void spi_set_cs_callback(spi_dev_t *dev, 
                         void (*cs_select)(uint8_t devid, bool selected));

/****************************************************************************
 * Name: spi_write_reg
 *
 * Description:
 *   Ghi 1 byte vào thanh ghi của SPI slave device
 *
 * Input Parameters:
 *   dev      - Con trỏ đến cấu trúc spi_dev_t
 *   reg_addr - Địa chỉ thanh ghi
 *   value    - Giá trị cần ghi
 *
 * Returned Value:
 *   SPI_OK on success; Mã lỗi âm on failure
 *
 ****************************************************************************/

int spi_write_reg(spi_dev_t *dev, uint8_t reg_addr, uint8_t value);

/****************************************************************************
 * Name: spi_read_reg
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

/****************************************************************************
 * Name: spi_read_regs
 *
 * Description:
 *   Đọc nhiều byte liên tiếp từ thanh ghi của SPI slave device
 *
 * Input Parameters:
 *   dev      - Con trỏ đến cấu trúc spi_dev_t
 *   reg_addr - Địa chỉ thanh ghi bắt đầu
 *   buffer   - Con trỏ buffer để lưu dữ liệu
 *   length   - Số byte cần đọc
 *
 * Returned Value:
 *   SPI_OK on success; Mã lỗi âm on failure
 *
 ****************************************************************************/

int spi_read_regs(spi_dev_t *dev, uint8_t reg_addr, 
                  uint8_t *buffer, size_t length);

/****************************************************************************
 * Name: spi_write_regs
 *
 * Description:
 *   Ghi nhiều byte liên tiếp vào thanh ghi của SPI slave device
 *
 * Input Parameters:
 *   dev      - Con trỏ đến cấu trúc spi_dev_t
 *   reg_addr - Địa chỉ thanh ghi bắt đầu
 *   buffer   - Con trỏ buffer chứa dữ liệu cần ghi
 *   length   - Số byte cần ghi
 *
 * Returned Value:
 *   SPI_OK on success; Mã lỗi âm on failure
 *
 ****************************************************************************/

int spi_write_regs(spi_dev_t *dev, uint8_t reg_addr, 
                   const uint8_t *buffer, size_t length);

/****************************************************************************
 * Name: spi_transfer
 *
 * Description:
 *   Thực hiện SPI transfer đồng thời ghi và đọc
 *
 * Input Parameters:
 *   dev      - Con trỏ đến cấu trúc spi_dev_t
 *   txbuffer - Buffer dữ liệu gửi đi
 *   rxbuffer - Buffer nhận dữ liệu
 *   length   - Số byte cần transfer
 *
 * Returned Value:
 *   SPI_OK on success; Mã lỗi âm on failure
 *
 ****************************************************************************/

int spi_transfer_user(spi_dev_t *dev, const uint8_t *txbuffer, 
                 uint8_t *rxbuffer, size_t length);

/****************************************************************************
 * Name: spi_select_bank
 *
 * Description:
 *   Chọn register bank (cho các sensor hỗ trợ bank switching)
 *
 * Input Parameters:
 *   dev  - Con trỏ đến cấu trúc spi_dev_t
 *   bank - Bank number (0-7)
 *
 * Returned Value:
 *   SPI_OK on success; Mã lỗi âm on failure
 *
 ****************************************************************************/

int spi_select_bank(spi_dev_t *dev, uint8_t bank);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_EXAMPLES_UAV_STATES_DRIVERS_COMM_SPI_SPI_H */