/****************************************************************************
 * boards/arm/stm32h7/weact-stm32h743/src/stm32_gpio.c
 *
 * GPIO safe-state initialization (CS, LED, power enables...)
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <stm32_gpio.h>

#include "board.h"
#include "weact-stm32h743.h"

void weact_gpio_init(void)
{
  /* LED PC0 - init low (off, assuming active-high) */
  stm32_configgpio(GPIO_LED_PC0);

  /* SPI1 IMU chip selects - init high (inactive) */
  stm32_configgpio(GPIO_SPI1_CS_IMU0);
  stm32_configgpio(GPIO_SPI1_CS_IMU1);
  stm32_configgpio(GPIO_SPI1_CS_IMU2);
  stm32_configgpio(GPIO_SPI1_CS_IMU3);

  /* Add any other GPIO init here (power rails, resets, debug pins) */
}