#include <nuttx/config.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <arch/board/board.h>

#include "stm32_gpio.h"

/* Struct mở rộng cho PC13 */
struct stm32_gpio_dev_s
{
  struct gpio_dev_s gpio;   /* Phần chung (upper-half nhìn thấy) */
  uint32_t pin;             /* Pin thật trên STM32 */
};

/* Hàm lower-half: đọc giá trị GPIO */
static int stm32_gpio_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  struct stm32_gpio_dev_s *priv = (struct stm32_gpio_dev_s *)dev;
  *value = stm32_gpioread(priv->pin);
  return OK;
}

/* Hàm lower-half: ghi giá trị GPIO */
static int stm32_gpio_write(FAR struct gpio_dev_s *dev, bool value)
{
  struct stm32_gpio_dev_s *priv = (struct stm32_gpio_dev_s *)dev;
  stm32_gpiowrite(priv->pin, value);
  return OK;
}

/* Không cần ngắt nên để NULL */
static int stm32_gpio_attach(FAR struct gpio_dev_s *dev, pin_interrupt_t handler)
{
  return OK;
}

/* Bảng function pointer cho upper-half */
static const struct gpio_operations_s stm32_gpio_ops =
{
  .go_read   = stm32_gpio_read,
  .go_write  = stm32_gpio_write,
  .go_attach = stm32_gpio_attach,
};

/* Instance cho PC13 */
static struct stm32_gpio_dev_s g_pc13led =
{
  .gpio =
  {
    .gp_pintype = GPIO_OUTPUT_PIN,
    .gp_ops     = &stm32_gpio_ops,
  },
  .pin = (GPIO_PORTC | GPIO_PIN13)   /* định nghĩa trong arch/stm32_gpio.h */
};

/* Hàm khởi tạo driver */
int stm32_pc13led_initialize(int minor)
{
  /* Cấu hình PC13 là output */
  stm32_configgpio(g_pc13led.pin);

  /* Đăng ký với upper-half */
  return gpio_pin_register(&g_pc13led.gpio, minor);
}
