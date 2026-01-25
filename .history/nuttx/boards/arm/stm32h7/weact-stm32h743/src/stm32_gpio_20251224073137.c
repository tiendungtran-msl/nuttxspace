#include <nuttx/config.h>
#include <stdbool.h>
#include "board.h"

#include <stm32_gpio.h>

/* 
 * Initialize board-specific GPIOs to a safe state.
 * Safe state = no SPI device selected, no contention, predictable boot.
 */

void weact_gpio_init(void)
{
  /* Configure IMU CS pins and set inactive (HIGH) */
  stm32_configgpio(GPIO_IMU0_CS);
  stm32_configgpio(GPIO_IMU1_CS);
  stm32_configgpio(GPIO_IMU2_CS);
  stm32_configgpio(GPIO_IMU3_CS);

  stm32_gpiowrite(GPIO_IMU0_CS, true);
  stm32_gpiowrite(GPIO_IMU1_CS, true);
  stm32_gpiowrite(GPIO_IMU2_CS, true);
  stm32_gpiowrite(GPIO_IMU3_CS, true);

  /* Add other board GPIO safe-state here later:
   * - sensor power enable
   * - GPS reset
   * - SD card power
   * - debug pins
   */
}