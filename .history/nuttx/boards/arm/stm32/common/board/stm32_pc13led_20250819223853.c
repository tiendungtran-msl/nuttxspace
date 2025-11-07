#include <nuttx/config.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/kmalloc.h>
#include <arch/board/board.h>
#include "stm32_gpio.h"

/* Cấu hình GPIO PC13 (LED trên Blackpill) */
#define GPIO_PC13 (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                   GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN13)

/* Lower-half operations (read/write/set/clear) */
static int pc13_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  *value = stm32_gpioread(GPIO_PC13);
  return OK;
}

static int pc13_write(FAR struct gpio_dev_s *dev, bool value)
{
  stm32_gpiowrite(GPIO_PC13, value);
  return OK;
}

/* ops struct */
static const struct gpio_operations_s g_pc13ops =
{
  .go_read  = pc13_read,
  .go_write = pc13_write,
};

/* Upper-half device instance */
static struct gpio_dev_s g_pc13dev =
{
  .gp_pintype = GPIO_OUTPUT_PIN,
  .gp_ops     = &g_pc13ops,
};

/* Public API: đăng ký driver */
int stm32_pc13led_initialize(int minor)
{
  /* Init hardware GPIO */
  stm32_configgpio(GPIO_PC13);

  /* Register vào VFS */
  return gpio_pin_register(&g_pc13dev, minor);
}
