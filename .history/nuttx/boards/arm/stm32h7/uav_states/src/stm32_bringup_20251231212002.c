/****************************************************************************
 * boards/arm/stm32h7/uav_states/src/stm32_bringup.c
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
#include <stdio.h>
#include "uav_states.h"
#include "stm32_gpio.h"

#ifdef CONFIG_SPI_DRIVER
#  include <nuttx/spi/spi_transfer.h>
#  include <nuttx/spi/spi.h>
#endif

#ifdef CONFIG_STM32H7_I2C
#  include "stm32_i2c.h"
#  include <nuttx/i2c/i2c_master.h>
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

  /* Đăng ký ngoại vi SPI cho ICM42688P */
  #ifdef CONFIG_STM32H7_SPI
  ret = stm32_spidev_initialize();
    if (ret < 0)
      {
        syslog(LOG_ERR, "ERROR: Failed to initialize SPI devices: %d\n", ret);
        return ret;
      }

  #ifdef CONFIG_DEV_GPIO
    ret = board_userled_initialize();
    if (ret < 0)
      {
        syslog(LOG_ERR, "ERROR: Failed to initialize GPIO LED: %d\n", ret);
      }
  #endif

  syslog(LOG_INFO, "SYSCLK: %ld Hz\n", STM32_SYSCLK_FREQUENCY);
  syslog(LOG_INFO, "HCLK  : %ld Hz\n", STM32_HCLK_FREQUENCY);
  syslog(LOG_INFO, "PCLK1 : %ld Hz\n", STM32_PCLK1_FREQUENCY);

  return OK;
}
