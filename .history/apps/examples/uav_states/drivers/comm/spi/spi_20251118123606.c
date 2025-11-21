/****************************************************************************
 * apps/examples/uav_states/drivers/comm/spi/spi.c
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

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include "spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * External Functions
 ****************************************************************************/

/* Board-specific CS control function (weak symbol, can be overridden) */

extern void board_spi1_icm_select(uint8_t id, bool selected) 
  __attribute__((weak));

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_cs_control
 *
 * Description:
 *   Lựa chọn hoặc bỏ lựa chọn CS (chip select) cho thiết bị SPI
 *
 ****************************************************************************/

static void spi_cs_control(spi_dev_t *dev, bool selected)
{
  if (dev->cs_select != NULL)
    {
      /* Use custom callback */
      dev->cs_select(dev->devid, selected);
    }
  else if (board_spi1_icm_select != NULL)
    {
      /* Use board-specific function */
      board_spi1_icm_select(dev->devid, selected);
    }
  else
    {
      /* No CS control available - rely on SPIIOC_TRANSFER deselect */
      _warn("WARNING: No CS control function available\n");
    }
}

/****************************************************************************
 * Name: spi_transfer_internal
 *
 * Description:
 *   Internal SPI transfer với retry và CS control
 *
 ****************************************************************************/

static int spi_transfer_internal(spi_dev_t *dev, 
                                  const uint8_t *txbuffer,
                                  uint8_t *rxbuffer, 
                                  size_t length)
{
  struct spi_trans_s trans;
  struct spi_sequence_s seq;
  int ret;
  int retries = SPI_MAX_RETRIES;

  if (!dev || !dev->initialized)
    {
      return SPI_ERR_INVALID;
    }

  if (!txbuffer || !rxbuffer || length == 0)
    {
      return SPI_ERR_INVALID;
    }

  /* CS assert */
  spi_cs_control(dev, true);
  usleep(SPI_CS_DELAY_US);

  /* Configure transfer */
  trans.deselect = false;  /* Manual CS control */
  trans.delay    = 0;
  trans.nwords   = (unsigned int)length;
  trans.txbuffer = (FAR void *)txbuffer;
  trans.rxbuffer = rxbuffer;

  seq.ntrans = 1;
  seq.trans  = &trans;

  /* Perform transfer with retry */
  while (retries-- > 0)
    {
      ret = ioctl(dev->fd, SPIIOC_TRANSFER, (unsigned long)&seq);
      if (ret >= 0)
        {
          break;
        }
      usleep(100);  /* Small delay before retry */
    }

  /* CS deassert */
  usleep(SPI_CS_DELAY_US);
  spi_cs_control(dev, false);

  if (ret < 0)
    {
      _err("ERROR: SPI transfer failed after %d retries: %d\n", 
           SPI_MAX_RETRIES, errno);
      return SPI_ERR_TRANSFER;
    }

  return SPI_OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_init
 ****************************************************************************/

int spi_init(spi_dev_t *dev, uint8_t bus, uint8_t devid)
{
  if (!dev)
    {
      return SPI_ERR_INVALID;
    }

  /* Initialize structure */
  memset(dev, 0, sizeof(spi_dev_t));
  dev->bus       = bus;

  /* Create device path */
  snprintf(dev->devpath, sizeof(dev->devpath), "/dev/spi%d", bus);

  /* Open SPI device */
  dev->fd = open(dev->devpath, O_RDWR);
  if (dev->fd < 0)
    {
      _err("ERROR: Failed to open %s: %d\n", dev->devpath, errno);
      return SPI_ERR_OPEN;
    }

  dev->initialized = true;
  dev->cs_select = board_spi1_icm_select;

  _info("SPI initialized: %s", dev->devpath);

  return SPI_OK;
}

/****************************************************************************
 * Name: spi_deinit
 ****************************************************************************/

int spi_deinit(spi_dev_t *dev)
{
  if (!dev || !dev->initialized)
    {
      return SPI_ERR_INVALID;
    }

  if (dev->fd >= 0)
    {
      close(dev->fd);
      dev->fd = -1;
    }

  dev->initialized = false;

  _info("SPI deinitialized: %s\n", dev->devpath);

  return SPI_OK;
}

/****************************************************************************
 * Name: spi_write_reg
 ****************************************************************************/

int spi_write_reg(spi_dev_t *dev, uint8_t reg_addr, uint8_t value)
{
  uint8_t tx[2];
  uint8_t rx[2];

  if (!dev || !dev->initialized)
    {
      return SPI_ERR_INVALID;
    }

  /* Prepare data: [reg_addr (write)][value] */
  tx[0] = reg_addr & 0x7F;  /* Clear read bit for write */
  tx[1] = value;

  memset(rx, 0, sizeof(rx));

  return spi_transfer_internal(dev, tx, rx, 2);
}

/****************************************************************************
 * Name: spi_read_reg
 ****************************************************************************/

int spi_read_reg(spi_dev_t *dev, uint8_t reg_addr, uint8_t *value)
{
  uint8_t tx[2];
  uint8_t rx[2];
  int ret;

  if (!dev || !dev->initialized || !value)
    {
      return SPI_ERR_INVALID;
    }

  /* Prepare: [reg_addr (read)][dummy] */
  tx[0] = reg_addr | SPI_READ_BIT;
  tx[1] = 0xFF;  /* Dummy byte */

  memset(rx, 0, sizeof(rx));

  ret = spi_transfer_internal(dev, tx, rx, 2);
  if (ret == SPI_OK)
    {
      *value = rx[1];  /* Data is in second byte */
    }

  return ret;
}

/****************************************************************************
 * Name: spi_read_regs
 ****************************************************************************/

int spi_read_regs(spi_dev_t *dev, uint8_t reg_addr, 
                  uint8_t *buffer, size_t length)
{
  uint8_t *tx;
  uint8_t *rx;
  int ret;
  size_t i;

  if (!dev || !dev->initialized || !buffer || length == 0)
    {
      syslog(LOG_ERR, "ERROR: Invalid parameters for spi_read_regs\n");
      return SPI_ERR_INVALID;
    }

  /* Allocate buffers */
  tx = (uint8_t *)malloc(length + 1);
  rx = (uint8_t *)malloc(length + 1);
  
  if (!tx || !rx)
    {
      free(tx);
      free(rx);
      syslog(LOG_ERR, "ERROR: Memory allocation failed in spi_read_regs\n");
      return SPI_ERR_INVALID;
    }

  /* Prepare: [reg_addr (read)][dummy bytes...] */
  tx[0] = reg_addr | SPI_READ_BIT;
  for (i = 1; i <= length; i++)
    {
      tx[i] = 0xFF;
    }

  memset(rx, 0, length + 1);

  ret = spi_transfer_internal(dev, tx, rx, length + 1);
  
  if (ret == SPI_OK)
    {
      /* Copy data (skip first byte which is dummy) */
      for (i = 0; i < length; i++)
        {
          buffer[i] = rx[i + 1];
        }
    }
  

  free(tx);
  free(rx);
  return ret;
}

/****************************************************************************
 * Name: spi_write_regs
 ****************************************************************************/

int spi_write_regs(spi_dev_t *dev, uint8_t reg_addr, 
                   const uint8_t *buffer, size_t length)
{
  uint8_t *tx;
  uint8_t *rx;
  int ret;
  size_t i;

  if (!dev || !dev->initialized || !buffer || length == 0)
    {
      return SPI_ERR_INVALID;
    }

  /* Allocate buffers */
  tx = (uint8_t *)malloc(length + 1);
  rx = (uint8_t *)malloc(length + 1);
  
  if (!tx || !rx)
    {
      free(tx);
      free(rx);
      return SPI_ERR_INVALID;
    }

  /* Prepare: [reg_addr (write)][data...] */
  tx[0] = reg_addr & 0x7F;
  for (i = 0; i < length; i++)
    {
      tx[i + 1] = buffer[i];
    }

  memset(rx, 0, length + 1);

  ret = spi_transfer_internal(dev, tx, rx, length + 1);

  free(tx);
  free(rx);
  return ret;
}

/****************************************************************************
 * Name: spi_transfer_ex
 ****************************************************************************/

int spi_transfer_ex(spi_dev_t *dev, const uint8_t *txbuffer, 
                 uint8_t *rxbuffer, size_t length)
{
  return spi_transfer_internal(dev, txbuffer, rxbuffer, length);
}