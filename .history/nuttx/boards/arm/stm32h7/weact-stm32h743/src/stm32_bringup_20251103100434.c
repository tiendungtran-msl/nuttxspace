/****************************************************************************
 * boards/arm/stm32h7/weact-stm32h743/src/stm32_bringup.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/types.h>
#include <syslog.h>
#include <errno.h>

#include <arch/board/board.h>

#include <nuttx/fs/fs.h>

#include "weact-stm32h743.h"

#include "stm32_gpio.h"

#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#endif

#ifdef CONFIG_STM32H7_SPI1
#  include "stm32_spi_icm.h"
#endif

#ifdef CONFIG_SPI_DRIVER
#  include <nuttx/spi/spi_transfer.h>
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y &&
 *   CONFIG_NSH_ARCHINIT:
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

  UNUSED(ret);

#ifdef CONFIG_SPI_DRIVER
  /* Register SPI1 as /dev/spi1 */

  ret = spi_register(spi1, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/spi1: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "SPI1 registered as /dev/spi1\n");
#endif

#ifdef CONFIG_STM32H7_SPI1
  /* Initialize SPI1 for ICM42688P sensors */

  syslog(LOG_INFO, "Initializing SPI1 for ICM42688P...\n");

  ret = board_spi1_icm_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: SPI1 initialization failed: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "SPI1 initialized successfully\n");
    }
#endif

#ifdef CONFIG_DEV_GPIO
  /* Initialize GPIO LED device */
  ret = stm32_gpio_led_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize GPIO LED: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n",  ret);
    }
#endif /* CONFIG_FS_PROCFS */

#if defined(CONFIG_FAT_DMAMEMORY)
  if (stm32_dma_alloc_init() < 0)
    {
      syslog(LOG_ERR, "DMA alloc FAILED");
    }
#endif

#ifdef HAVE_SDIO
  /* Initialize the SDIO block driver */

  ret = stm32_sdio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize MMC/SD driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

  // Trong stm32_bringup.c hoặc NSH command
  syslog(LOG_INFO, "SYSCLK: %ld Hz\n", STM32_SYSCLK_FREQUENCY);
  syslog(LOG_INFO, "HCLK: %ld Hz\n", STM32_HCLK_FREQUENCY);
  syslog(LOG_INFO, "PCLK1: %ld Hz\n", STM32_PCLK1_FREQUENCY);

  return OK;
}
