#include <nuttx/config.h>
#include <nuttx/board.h>
#include <stdbool.h>
#include "board.h"
#include <stm32_gpio.h>

#define BOARD_NLEDS 1
#define LED_PC0     0

/* Nếu LED active-low, đổi macro này thành (!on) */
static inline void led_write(bool on)
{
  stm32_gpiowrite(GPIO_USERLED_PC0, on);
}

uint32_t board_userled_initialize(void)
{
  stm32_configgpio(GPIO_USERLED_PC0);
  led_write(false);
  return BOARD_NLEDS;
}

void board_userled(int led, bool on)
{
  if (led == LED_PC0)
    {
      led_write(on);
    }
}

void board_userled_all(uint32_t ledset)
{
  led_write((ledset & (1 << LED_PC0)) != 0);
}