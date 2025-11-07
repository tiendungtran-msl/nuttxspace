/****************************************************************************
 * boards/arm/stm32h7/weact-stm32h743/src/stm32_gpio_led.c
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <syslog.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/ioexpander/gpio.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "weact-stm32h743.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO LED on PC0 */
#define GPIO_LED  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_PULLUP | \
                   GPIO_SPEED_50MHz | GPIO_PORTC | GPIO_PIN0)

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

static int gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct stm32gpio_dev_s *stm32gpio = 
    (FAR struct stm32gpio_dev_s *)dev;

  DEBUGASSERT(stm32gpio != NULL && value != NULL);
  DEBUGASSERT(stm32gpio->pinset != 0);

  *value = stm32_gpioread(stm32gpio->pinset);
  return OK;
}

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

int stm32_gpio_led_initialize(void)
{
  int ret;

  /* Configure GPIO as output */
  
  stm32_configgpio(GPIO_LED);
  stm32_gpiowrite(GPIO_LED, false);

  /* Register the GPIO driver */

  ret = gpio_pin_register_byname(&g_gpout_led.gpio, "led");
  if (ret < 0)
    {
      gpioerr("ERROR: gpio_pin_register failed: %d\n", ret);
      return ret;
    }

  gpioinfo("GPIO LED initialized successfully\n");
  return OK;
}

#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */