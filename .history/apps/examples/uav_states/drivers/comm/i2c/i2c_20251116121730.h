/****************************************************************************
 * apps/examples/uav_states/drivers/comm/i2c/i2c.h
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

#ifndef __APPS_EXAMPLES_UAV_STATES_DRIVERS_COMM_I2C_I2C_H
#define __APPS_EXAMPLES_UAV_STATES_DRIVERS_COMM_I2C_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C Return Codes */

#define I2C_OK              0
#define I2C_ERR_INIT       -1
#define I2C_ERR_OPEN       -2
#define I2C_ERR_IOCTL      -3
#define I2C_ERR_WRITE      -4
#define I2C_ERR_READ       -5
#define I2C_ERR_TRANSFER   -6
#define I2C_ERR_INVALID    -7

/* Default I2C Configuration */

#define I2C_FREQUENCY  400000  /* 400kHz (Fast Mode) */
#define I2C_MAX_RETRIES        3

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* I2C Device Handle */

typedef struct i2c_dev_s
{
  int fd;                    /* File descriptor */
  uint8_t addr;              /* 7-bit I2C slave address */
  uint32_t frequency;        /* I2C bus frequency */
  char devpath[32];          /* Device path (e.g., "/dev/i2c0") */
  bool initialized;          /* Initialization status */
} i2c_dev_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: i2c_init
 *
 * Description:
 *   Khởi tạo I2C device từ user space
 *
 * Input Parameters:
 *   dev       - Con trỏ đến cấu trúc i2c_dev_t
 *   bus       - Số bus I2C (0, 1, 2, ...)
 *   addr      - Địa chỉ slave 7-bit
 *   frequency - Tần số I2C (Hz), 0 = sử dụng mặc định
 *
 * Returned Value:
 *   I2C_OK on success; Mã lỗi âm on failure
 *
 ****************************************************************************/

int i2c_init(i2c_dev_t *dev, uint8_t bus, uint8_t addr, uint32_t frequency);

/****************************************************************************
 * Name: i2c_deinit
 *
 * Description:
 *   Hủy khởi tạo và đóng I2C device
 *
 * Input Parameters:
 *   dev - Con trỏ đến cấu trúc i2c_dev_t
 *
 * Returned Value:
 *   I2C_OK on success; Mã lỗi âm on failure
 *
 ****************************************************************************/

int i2c_deinit(i2c_dev_t *dev);

/****************************************************************************
 * Name: i2c_write_reg
 *
 * Description:
 *   Ghi 1 byte vào thanh ghi của slave device
 *
 * Input Parameters:
 *   dev      - Con trỏ đến cấu trúc i2c_dev_t
 *   reg_addr - Địa chỉ thanh ghi
 *   value    - Giá trị cần ghi
 *
 * Returned Value:
 *   I2C_OK on success; Mã lỗi âm on failure
 *
 ****************************************************************************/

int i2c_write_reg(i2c_dev_t *dev, uint8_t reg_addr, uint8_t value);

/****************************************************************************
 * Name: i2c_read_reg
 *
 * Description:
 *   Đọc 1 byte từ thanh ghi của slave device
 *
 * Input Parameters:
 *   dev      - Con trỏ đến cấu trúc i2c_dev_t
 *   reg_addr - Địa chỉ thanh ghi
 *   value    - Con trỏ để lưu giá trị đọc được
 *
 * Returned Value:
 *   I2C_OK on success; Mã lỗi âm on failure
 *
 ****************************************************************************/

int i2c_read_reg(i2c_dev_t *dev, uint8_t reg_addr, uint8_t *value);

/****************************************************************************
 * Name: i2c_read_regs
 *
 * Description:
 *   Đọc nhiều byte liên tiếp từ thanh ghi của slave device
 *
 * Input Parameters:
 *   dev      - Con trỏ đến cấu trúc i2c_dev_t
 *   reg_addr - Địa chỉ thanh ghi bắt đầu
 *   buffer   - Con trỏ buffer để lưu dữ liệu
 *   length   - Số byte cần đọc
 *
 * Returned Value:
 *   I2C_OK on success; Mã lỗi âm on failure
 *
 ****************************************************************************/

int i2c_read_regs(i2c_dev_t *dev, uint8_t reg_addr, 
                  uint8_t *buffer, size_t length);

/****************************************************************************
 * Name: i2c_write_regs
 *
 * Description:
 *   Ghi nhiều byte liên tiếp vào thanh ghi của slave device
 *
 * Input Parameters:
 *   dev      - Con trỏ đến cấu trúc i2c_dev_t
 *   reg_addr - Địa chỉ thanh ghi bắt đầu
 *   buffer   - Con trỏ buffer chứa dữ liệu cần ghi
 *   length   - Số byte cần ghi
 *
 * Returned Value:
 *   I2C_OK on success; Mã lỗi âm on failure
 *
 ****************************************************************************/

int i2c_write_regs(i2c_dev_t *dev, uint8_t reg_addr, 
                   const uint8_t *buffer, size_t length);

/****************************************************************************
 * Name: i2c_probe
 *
 * Description:
 *   Kiểm tra xem slave device có tồn tại trên bus không
 *
 * Input Parameters:
 *   dev - Con trỏ đến cấu trúc i2c_dev_t
 *
 * Returned Value:
 *   true nếu device phản hồi; false nếu không
 *
 ****************************************************************************/

bool i2c_probe(i2c_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_EXAMPLES_UAV_STATES_DRIVERS_COMM_I2C_I2C_H */