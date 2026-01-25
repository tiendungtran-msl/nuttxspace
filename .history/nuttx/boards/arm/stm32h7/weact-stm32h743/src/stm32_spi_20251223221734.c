/****************************************************************************
 * boards/arm/stm32h7/weact-stm32h743/src/stm32_spi.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"

#include "weact-stm32h743.h"
#include <arch/board/board.h>

#ifdef CONFIG_STM32H7_SPI1
#  include "stm32_spi_icm.h"
#endif

#ifdef CONFIG_STM32H7_SPI

#define GPIO_IMU0_CS  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN4)
#define GPIO_IMU1_CS  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_100MHz|GPIO_PORTD1GPIO_PIN10)
#define GPIO_IMU2_CS  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN11)
#define GPIO_IMU3_CS  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN12)

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/* Khai báo các chân select */
static inline void imu_cs_init(void)
{
  stm32_configgpio(GPIO_IMU0_CS); stm32_gpiowrite(GPIO_IMU0_CS, true);
  stm32_configgpio(GPIO_IMU1_CS); stm32_gpiowrite(GPIO_IMU1_CS, true);
  stm32_configgpio(GPIO_IMU2_CS); stm32_gpiowrite(GPIO_IMU2_CS, true);
  stm32_configgpio(GPIO_IMU3_CS); stm32_gpiowrite(GPIO_IMU3_CS, true);
}

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the weact-stm32h743
 *   board.
 *
 ****************************************************************************/

void stm32_spidev_initialize(void)
{
#ifdef CONFIG_STM32H7_SPI1
  /* Initialize ICM42688P CS pins */

  stm32_spidev_icm_initialize();
#endif

#ifdef CONFIG_LCD_ST7735
  stm32_configgpio(GPIO_LCD_CS);    /* ST7735 chip select */
#endif
}

/****************************************************************************
 * Name:  stm32_spi1select and stm32_spi1status
 *
 * Description:
 *   The external functions, stm32_spi1select and stm32_spi1status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *  (including stm32_spibus_initialize()) are provided by common STM32 logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1select() and stm32_spi1status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI1
void stm32_spi1select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  /* active-low CS */
  bool inactive = !selected;

  switch (devid)
  {
    case SPIDEV_IMU0: stm32_gpiowrite(GPIO_IMU0_CS, inactive); break;
    case SPIDEV_IMU1: stm32_gpiowrite(GPIO_IMU1_CS, inactive); break;
    case SPIDEV_IMU2: stm32_gpiowrite(GPIO_IMU2_CS, inactive); break;
    case SPIDEV_IMU3: stm32_gpiowrite(GPIO_IMU3_CS, inactive); break;
    default: break;
  }
}

uint8_t stm32_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}
#endif

/****************************************************************************
 * Name: stm32_spi4cmddata
 *
 * Description:
 *   This is an implementation of the cmddata method of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 * Input Parameters:
 *
 *   spi - SPI device that controls the bus the device that requires the CMD/
 *         DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *         to select cmd or data.  NOTE:  This design restricts, for example,
 *         one one SPI display per SPI bus.
 *   cmd - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_STM32H7_SPI4
int stm32_spi4cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#ifdef CONFIG_LCD_ST7735
  if (devid == SPIDEV_DISPLAY(0))
    {
      /*  This is the Data/Command control pad which determines whether the
       *  data bits are data or a command.
       */

      stm32_gpiowrite(GPIO_LCD_DC, !cmd);
      return OK;
    }
#endif

  return -ENODEV;
}
#endif
#endif

#endif /* CONFIG_STM32H7_SPI */