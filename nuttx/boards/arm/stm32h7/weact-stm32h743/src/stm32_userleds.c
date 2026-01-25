/****************************************************************************
 * boards/arm/stm32h7/weact-stm32h743/src/stm32_userleds.c
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <stdbool.h>
#include <stm32_gpio.h>

#include "board.h"

#ifdef CONFIG_USERLED

/* Assume LED PC0 is active-high.  If active-low, invert logic.  */

uint32_t board_userled_initialize(void)
{
  stm32_configgpio(GPIO_LED_PC0);
  return BOARD_NLEDS;
}

void board_userled(int led, bool ledon)
{
  if (led == BOARD_LED_PC0)
    {
      stm32_gpiowrite(GPIO_LED_PC0, ledon);
    }
}

void board_userled_all(uint32_t ledset)
{
  board_userled(BOARD_LED_PC0, (ledset & (1 << BOARD_LED_PC0)) != 0);
}

#endif /* CONFIG_USERLED */