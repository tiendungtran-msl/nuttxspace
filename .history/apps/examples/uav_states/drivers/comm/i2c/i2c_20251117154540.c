/****************************************************************************
 * apps/examples/uav_states/drivers/comm/i2c/i2c.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>

#include "i2c.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_transfer_with_retry
 *
 * Description:
 *   Thực hiện I2C transfer với retry mechanism
 *
 ****************************************************************************/

static int i2c_transfer_with_retry(i2c_dev_t *dev, 
                                    struct i2c_msg_s *msgs, 
                                    int num_msgs)
{
  struct i2c_transfer_s xfer;
  int ret;
  int retries = I2C_MAX_RETRIES;

  if (!dev || !dev->initialized)
    {
      return I2C_ERR_INVALID;
    }

  xfer.msgv = msgs;
  xfer.msgc = num_msgs;

  while (retries-- > 0)
    {
      ret = ioctl(dev->fd, I2CIOC_TRANSFER, (unsigned long)&xfer);
      if (ret >= 0)
        {
          return I2C_OK;
        }

      /* Delay trước khi retry */
      usleep(1000);
    }

  _err("ERROR: I2C transfer failed after %d retries: %d\n", 
       I2C_MAX_RETRIES, errno);
  return I2C_ERR_TRANSFER;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_init
 ****************************************************************************/

int i2c_init(i2c_dev_t *dev, uint8_t bus, uint8_t addr)
{
  int ret;

  if (!dev)
    {
      return I2C_ERR_INVALID;
    }

  /* Kiểm tra địa chỉ I2C hợp lệ (7-bit) */
  if (addr > 0x7F)
    {
      _err("ERROR: Invalid I2C address 0x%02X\n", addr);
      return I2C_ERR_INVALID;
    }

  /* Khởi tạo cấu trúc */
  memset(dev, 0, sizeof(i2c_dev_t));
  dev->addr = addr;

  /* Tạo device path */
  snprintf(dev->devpath, sizeof(dev->devpath), "/dev/i2c%d", bus);

  /* Mở I2C device */
  dev->fd = open(dev->devpath, O_RDWR);
  if (dev->fd < 0)
    {
      _err("ERROR: Failed to open %s: %d\n", dev->devpath, errno);
      return I2C_ERR_OPEN;
    }

  dev->initialized = true;

  _info("I2C initialized: %s, addr=0x%02X, freq=%u Hz\n",
        dev->devpath, dev->addr);

  return I2C_OK;
}

/****************************************************************************
 * Name: i2c_deinit
 ****************************************************************************/

int i2c_deinit(i2c_dev_t *dev)
{
  if (!dev || !dev->initialized)
    {
      return I2C_ERR_INVALID;
    }

  if (dev->fd >= 0)
    {
      close(dev->fd);
      dev->fd = -1;
    }

  dev->initialized = false;

  _info("I2C deinitialized: %s\n", dev->devpath);

  return I2C_OK;
}

/****************************************************************************
 * Name: i2c_write_reg
 ****************************************************************************/

int i2c_write_reg(i2c_dev_t *dev, uint8_t reg_addr, uint8_t value)
{
  struct i2c_msg_s msg;
  uint8_t buffer[2];

  if (!dev || !dev->initialized)
    {
      return I2C_ERR_INVALID;
    }

  /* Chuẩn bị dữ liệu: [reg_addr][value] */
  buffer[0] = reg_addr;
  buffer[1] = value;

  /* Cấu hình I2C message */
  msg.frequency = dev->frequency;
  msg.addr      = dev->addr;
  msg.flags     = 0;                /* Write operation */
  msg.buffer    = buffer;
  msg.length    = 2;

  return i2c_transfer_with_retry(dev, &msg, 1);
}

/****************************************************************************
 * Name: i2c_read_reg
 ****************************************************************************/

int i2c_read_reg(i2c_dev_t *dev, uint8_t reg_addr, uint8_t *value)
{
  struct i2c_msg_s msgs[2];

  if (!dev || !dev->initialized || !value)
    {
      return I2C_ERR_INVALID;
    }

  /* Message 1: Ghi địa chỉ thanh ghi */
  msgs[0].frequency = dev->frequency;
  msgs[0].addr      = dev->addr;
  msgs[0].flags     = 0;
  msgs[0].buffer    = &reg_addr;
  msgs[0].length    = 1;

  /* Message 2: Đọc dữ liệu */
  msgs[1].frequency = dev->frequency;
  msgs[1].addr      = dev->addr;
  msgs[1].flags     = I2C_M_READ;
  msgs[1].buffer    = value;
  msgs[1].length    = 1;

  return i2c_transfer_with_retry(dev, msgs, 2);
}

/****************************************************************************
 * Name: i2c_read_regs
 ****************************************************************************/

int i2c_read_regs(i2c_dev_t *dev, uint8_t reg_addr, 
                  uint8_t *buffer, size_t length)
{
  struct i2c_msg_s msgs[2];

  if (!dev || !dev->initialized || !buffer || length == 0)
    {
      return I2C_ERR_INVALID;
    }

  /* Message 1: Ghi địa chỉ thanh ghi bắt đầu */
  msgs[0].frequency = dev->frequency;
  msgs[0].addr      = dev->addr;
  msgs[0].flags     = 0;
  msgs[0].buffer    = &reg_addr;
  msgs[0].length    = 1;

  /* Message 2: Đọc nhiều byte */
  msgs[1].frequency = dev->frequency;
  msgs[1].addr      = dev->addr;
  msgs[1].flags     = I2C_M_READ;
  msgs[1].buffer    = buffer;
  msgs[1].length    = length;

  return i2c_transfer_with_retry(dev, msgs, 2);
}

/****************************************************************************
 * Name: i2c_write_regs
 ****************************************************************************/

int i2c_write_regs(i2c_dev_t *dev, uint8_t reg_addr, 
                   const uint8_t *buffer, size_t length)
{
  struct i2c_msg_s msg;
  uint8_t *txbuffer;
  int ret;

  if (!dev || !dev->initialized || !buffer || length == 0)
    {
      return I2C_ERR_INVALID;
    }

  /* Allocate buffer: [reg_addr][data...] */
  txbuffer = (uint8_t *)malloc(length + 1);
  if (!txbuffer)
    {
      return I2C_ERR_INVALID;
    }

  txbuffer[0] = reg_addr;
  memcpy(&txbuffer[1], buffer, length);

  /* Cấu hình I2C message */
  msg.frequency = dev->frequency;
  msg.addr      = dev->addr;
  msg.flags     = 0;
  msg.buffer    = txbuffer;
  msg.length    = length + 1;

  ret = i2c_transfer_with_retry(dev, &msg, 1);

  free(txbuffer);
  return ret;
}

/****************************************************************************
 * Name: i2c_probe
 ****************************************************************************/

bool i2c_probe(i2c_dev_t *dev)
{
  struct i2c_msg_s msg;
  uint8_t dummy = 0;

  if (!dev || !dev->initialized)
    {
      return false;
    }

  /* Thử đọc 0 byte để kiểm tra ACK */
  msg.frequency = dev->frequency;
  msg.addr      = dev->addr;
  msg.flags     = I2C_M_READ;
  msg.buffer    = &dummy;
  msg.length    = 0;

  if (i2c_transfer_with_retry(dev, &msg, 1) == I2C_OK)
    {
      _info("I2C device found at address 0x%02X\n", dev->addr);
      return true;
    }

  return false;
}