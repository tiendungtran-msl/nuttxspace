/****************************************************************************
 * boards/arm/stm32h7/weact-stm32h743/src/stm32_gpio_led.c
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

#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "weact-stm32h743.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO LED Configuration */
#define GPIO_LED  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_PULLUP | \
                   GPIO_SPEED_50MHz | GPIO_PORTC | GPIO_PIN0)

/* Device path */
#define GPIO_LED_DEVPATH "/dev/gpio_led"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint32_t pinset;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int gpout_write(FAR struct gpio_dev_s *dev, bool value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s gpout_ops =
{
  .go_read   = gpout_read,
  .go_write  = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
};

/* GPIO LED device */
static struct stm32gpio_dev_s g_gpout_led =
{
  .gpio =
    {
      .gp_pintype = GPIO_OUTPUT_PIN,
      .gp_ops     = &gpout_ops,
    },
  .pinset = GPIO_LED,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpout_read
 *
 * Description:
 *   Read the current state of the GPIO output pin
 *
 ****************************************************************************/

static int gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct stm32gpio_dev_s *stm32gpio = 
    (FAR struct stm32gpio_dev_s *)dev;

  DEBUGASSERT(stm32gpio != NULL && value != NULL);
  DEBUGASSERT(stm32gpio->pinset != 0);

  *value = stm32_gpioread(stm32gpio->pinset);
  return OK;
}

/****************************************************************************
 * Name: gpout_write
 *
 * Description:
 *   Set the state of the GPIO output pin
 *
 ****************************************************************************/

static int gpout_write(FAR struct gpio_dev_s *dev, bool value)
{
  FAR struct stm32gpio_dev_s *stm32gpio = 
    (FAR struct stm32gpio_dev_s *)dev;

  DEBUGASSERT(stm32gpio != NULL);
  DEBUGASSERT(stm32gpio->pinset != 0);

  stm32_gpiowrite(stm32gpio->pinset, value);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_gpio_led_initialize
 *
 * Description:
 *   Initialize GPIO LED device driver
 *
 ****************************************************************************/

int stm32_gpio_led_initialize(void)
{
  int ret;

  /* Configure the GPIO pin */
  stm32_configgpio(GPIO_LED);
  
  /* Set initial state to OFF (LOW) */
  stm32_gpiowrite(GPIO_LED, false);

  /* Register the GPIO device */
  ret = gpio_register(GPIO_LED_DEVPATH, &g_gpout_led.gpio);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register %s: %d\n",
             GPIO_LED_DEVPATH, ret);
      return ret;
    }

  syslog(LOG_INFO, "GPIO LED registered at %s\n", GPIO_LED_DEVPATH);
  return OK;
}

#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */