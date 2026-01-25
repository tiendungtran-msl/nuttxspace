/*****************************************************************************
 * boards/arm/stm32h7/uav_states/src/stm32_userled.c

 * Cấu hình LED test cho board UAV States
 * *****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "uav_states.h"
#include ""../include/board.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

 void userled_initialize(void)
 {
    /* Cấu hình chân GPIO cho LED trạng thái */
    stm32_configgpio(GPIO_LED_STATUS);
 }

 void ctrl_userled(int led, bool state)
 {
    if (led == 0)
      {
        /* Bật hoặc tắt LED trạng thái */
        stm32_gpiowrite(GPIO_LED_STATUS, state);
      }
 }