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

#include <nuttx/ioexpander/gpio.h>

#include "config.h"
#include "gpio_driver.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_gpio_fd = -1;
static bool g_gpio_initialized = false;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int gpio_pc13_init(void)
{
  int ret;

  if (g_gpio_initialized)
    {
      printf("GPIO PC13 already initialized\n");
      return PC13_OK;
    }

  /* Open GPIO device */
  g_gpio_fd = open("/dev/gpio0", O_RDWR);
  if (g_gpio_fd < 0)
    {
      printf("ERROR: Failed to open GPIO device: %d\n", errno);
      return PC13_ERROR;
    }

  /* Configure as output */
  ret = ioctl(g_gpio_fd, GPIOC_SETPINTYPE, GPIO_OUTPUT_PIN);
  if (ret < 0)
    {
      printf("ERROR: Failed to configure GPIO as output: %d\n", errno);
      close(g_gpio_fd);
      g_gpio_fd = -1;
      return PC13_ERROR;
    }

  g_gpio_initialized = true;
  printf("GPIO PC13 initialized as output\n");
  
  return PC13_OK;
}

int gpio_pc13_deinit(void)
{
  if (!g_gpio_initialized)
    {
      return PC13_OK;
    }

  if (g_gpio_fd >= 0)
    {
      close(g_gpio_fd);
      g_gpio_fd = -1;
    }

  g_gpio_initialized = false;
  printf("GPIO PC13 deinitialized\n");
  
  return PC13_OK;
}

int gpio_pc13_set(bool state)
{
  int ret;

  if (!g_gpio_initialized || g_gpio_fd < 0)
    {
      printf("ERROR: GPIO not initialized\n");
      return PC13_ERROR;
    }

  ret = ioctl(g_gpio_fd, GPIOC_WRITE, (unsigned long)state);
  if (ret < 0)
    {
      printf("ERROR: Failed to set GPIO state: %d\n", errno);
      return PC13_ERROR;
    }

  return PC13_OK;
}

bool gpio_pc13_get(void)
{
  int ret;
  bool state = false;

  if (!g_gpio_initialized || g_gpio_fd < 0)
    {
      printf("ERROR: GPIO not initialized\n");
      return false;
    }

  ret = ioctl(g_gpio_fd, GPIOC_READ, (unsigned long)&state);
  if (ret < 0)
    {
      printf("ERROR: Failed to read GPIO state: %d\n", errno);
      return false;
    }

  return state;
}

int gpio_pc13_toggle(void)
{
  bool current_state;
  
  current_state = gpio_pc13_get();
  return gpio_pc13_set(!current_state);
}