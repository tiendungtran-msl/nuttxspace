/****************************************************************************
 * src/gpio_driver.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

/* STM32 specific headers */
#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <chip.h>
#include <stm32.h>

#include "config.h"
#include "gpio_driver.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_gpio_initialized = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Direct STM32 GPIO access method */
static int gpio_pc13_init_direct(void)
{
  /* Enable GPIOC clock */
  stm32_configgpio(PC13_GPIO_CONFIG);
  
  printf("GPIO PC13 configured directly via STM32 HAL\n");
  return PC13_OK;
}

static int gpio_pc13_set_direct(bool state)
{
  stm32_gpiowrite(PC13_GPIO_CONFIG, state);
  return PC13_OK;
}

static bool gpio_pc13_get_direct(void)
{
  return stm32_gpioread(PC13_GPIO_CONFIG);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int gpio_pc13_init(void)
{
  if (g_gpio_initialized)
    {
      printf("GPIO PC13 already initialized\n");
      return PC13_OK;
    }

  /* Method 1: Direct STM32 GPIO access (recommended for PC13) */
  int ret = gpio_pc13_init_direct();
  if (ret == PC13_OK)
    {
      g_gpio_initialized = true;
      printf("GPIO PC13 initialized successfully (direct method)\n");
    }

  return ret;
}

int gpio_pc13_deinit(void)
{
  if (!g_gpio_initialized)
    {
      return PC13_OK;
    }

  /* Reset GPIO to input mode */
  stm32_configgpio(GPIO_INPUT | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN13);
  
  g_gpio_initialized = false;
  printf("GPIO PC13 deinitialized\n");
  
  return PC13_OK;
}

int gpio_pc13_set(bool state)
{
  if (!g_gpio_initialized)
    {
      printf("ERROR: GPIO not initialized\n");
      return PC13_ERROR;
    }

  return gpio_pc13_set_direct(state);
}

bool gpio_pc13_get(void)
{
  if (!g_gpio_initialized)
    {
      printf("ERROR: GPIO not initialized\n");
      return false;
    }

  return gpio_pc13_get_direct();
}

int gpio_pc13_toggle(void)
{
  bool current_state = gpio_pc13_get();
  return gpio_pc13_set(!current_state);
}