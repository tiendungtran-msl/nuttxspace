/****************************************************************************
 * examples/stm32_gpio_control/board_setup/stm32f411_gpio_setup.c
 *
 * Board GPIO Device Setup Implementation
 * Tạo và register GPIO device files cho STM32F411 BlackPill
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/ioexpander/gpio.h>
#include <arch/board/board.h>

#ifdef CONFIG_STM32_STM32F4XX
#  include "stm32.h"
#  include "stm32_gpio.h"
#endif

#include "stm32f411_gpio_setup.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* GPIO device structure for PC13 */
static struct gpio_dev_s g_gpio_pc13_dev;
static bool g_gpio_initialized = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_pc13_read
 *
 * Description:
 *   Read GPIO PC13 value - Interface function cho NuttX GPIO framework
 ****************************************************************************/

static int gpio_pc13_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  DEBUGASSERT(dev != NULL && value != NULL);
  
  /* 
   * Đọc GPIO value từ STM32 hardware registers
   * stm32_gpioread() là low-level function từ STM32 HAL
   */
  *value = stm32_gpioread(BOARD_GPIO_PC13_PIN);
  
  gpioinfo("PC13 read: %s\n", *value ? "HIGH" : "LOW");
  return OK;
}

/****************************************************************************
 * Name: gpio_pc13_write
 *
 * Description:
 *   Write GPIO PC13 value - Interface function cho NuttX GPIO framework
 ****************************************************************************/

static int gpio_pc13_write(FAR struct gpio_dev_s *dev, bool value)
{
  DEBUGASSERT(dev != NULL);
  
  /* 
   * Ghi GPIO value vào STM32 hardware registers
   * stm32_gpiowrite() là low-level function từ STM32 HAL
   */
  stm32_gpiowrite(BOARD_GPIO_PC13_PIN, value);
  
  gpioinfo("PC13 write: %s\n", value ? "HIGH" : "LOW");
  return OK;
}

/****************************************************************************
 * Name: gpio_pc13_attach
 *
 * Description:
 *   Attach interrupt handler (not used for output pin)
 ****************************************************************************/

static int gpio_pc13_attach(FAR struct gpio_dev_s *dev,
                           pin_interrupt_t callback)
{
  gpioinfo("PC13 attach interrupt (not supported for output)\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: gpio_pc13_enable
 *
 * Description:
 *   Enable interrupt (not used for output pin)
 ****************************************************************************/

static int gpio_pc13_enable(FAR struct gpio_dev_s *dev, bool enable)
{
  gpioinfo("PC13 enable interrupt (not supported for output)\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: gpio_pc13_setpintype
 *
 * Description:
 *   Set GPIO pin type (input/output)
 ****************************************************************************/

static int gpio_pc13_setpintype(FAR struct gpio_dev_s *dev,
                                enum gpio_pintype_e pintype)
{
  uint32_t pinconfig;
  
  DEBUGASSERT(dev != NULL);
  
  /* Configure pin based on requested type */
  switch (pintype)
    {
      case GPIO_INPUT_PIN:
        pinconfig = GPIO_INPUT | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN13;
        break;
        
      case GPIO_OUTPUT_PIN:
        pinconfig = BOARD_GPIO_PC13_PIN;  /* Default output configuration */
        break;
        
      default:
        gpioerr("Unsupported pin type: %d\n", pintype);
        return -EINVAL;
    }
  
  /* 
   * Reconfigure GPIO pin với STM32 hardware
   * stm32_configgpio() cấu hình GPIO registers
   */
  int ret = stm32_configgpio(pinconfig);
  if (ret < 0)
    {
      gpioerr("Failed to configure PC13 as pintype %d\n", pintype);
      return ret;
    }
  
  gpioinfo("PC13 configured as %s\n", 
           (pintype == GPIO_INPUT_PIN) ? "INPUT" : "OUTPUT");
  
  return OK;
}

/****************************************************************************
 * GPIO Operations Structure
 * Định nghĩa interface functions cho NuttX GPIO framework
 ****************************************************************************/

static const struct gpio_operations_s g_gpio_pc13_ops =
{
  .go_read       = gpio_pc13_read,
  .go_write      = gpio_pc13_write,
  .go_attach     = gpio_pc13_attach,
  .go_enable     = gpio_pc13_enable,
  .go_setpintype = gpio_pc13_setpintype,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_gpio_setup
 ****************************************************************************/

int board_gpio_setup(void)
{
  int ret;
  
  if (g_gpio_initialized)
    {
      gpioinfo("GPIO devices already initialized\n");
      return OK;
    }
  
  gpioinfo("Setting up STM32F411 GPIO devices...\n");
  
  /* 
   * Bước 1: Configure STM32 GPIO hardware
   * Setup PC13 như output pin với proper electrical characteristics
   */
  ret = stm32_configgpio(BOARD_GPIO_PC13_PIN);
  if (ret < 0)
    {
      gpioerr("Failed to configure PC13 GPIO pin: %d\n", ret);
      return ret;
    }
  
  /* Set initial state - LED OFF (PC13 high for BlackPill) */
  stm32_gpiowrite(BOARD_GPIO_PC13_PIN, true);
  
  /* 
   * Bước 2: Initialize GPIO device structure
   * Liên kết hardware với NuttX GPIO framework
   */
  g_gpio_pc13_dev.gp_pintype = GPIO_OUTPUT_PIN;
  g_gpio_pc13_dev.gp_ops     = &g_gpio_pc13_ops;
  
  /* 
   * Bước 3: Register GPIO device với NuttX device framework
   * Tạo character device file /dev/gpio13
   * gpio_register() là NuttX framework function
   */
  ret = gpio_register(&g_gpio_pc13_dev, 13);  /* Creates /dev/gpio13 */
  if (ret < 0)
    {
      gpioerr("Failed to register GPIO device 13: %d\n", ret);
      return ret;
    }
  
  /* 
   * Bước 4: Create additional device aliases
   * Một số applications expect different naming conventions
   */
  ret = gpio_register(&g_gpio_pc13_dev, 113); /* Creates /dev/gpio113 as backup */
  if (ret < 0)
    {
      gpioinfo("Could not create backup GPIO device (non-critical): %d\n", ret);
      /* Continue anyway */
    }
  
  g_gpio_initialized = true;
  
  gpioinfo("STM32F411 GPIO setup completed successfully\n");
  gpioinfo("Created devices: /dev/gpio13\n");
  gpioinfo("PC13 configured as OUTPUT, initial state: HIGH (LED OFF)\n");
  
  return OK;
}

/****************************************************************************
 * Name: board_gpio_cleanup
 ****************************************************************************/

int board_gpio_cleanup(void)
{
  if (!g_gpio_initialized)
    {
      return OK;
    }
  
  /* 
   * Unregister GPIO devices
   * Note: gpio_unregister() may not be available in all NuttX versions
   */
#ifdef CONFIG_GPIO_UNREGISTER
  gpio_unregister(13);
  gpio_unregister(113);
#endif
  
  /* Reset GPIO to safe state */
  stm32_gpiowrite(BOARD_GPIO_PC13_PIN, true); /* LED OFF */
  
  g_gpio_initialized = false;
  
  gpioinfo("GPIO cleanup completed\n");
  
  return OK;
}

/****************************************************************************
 * Name: board_gpio_initialize (Optional board integration)
 *
 * Description:
 *   Initialize board GPIO - called from board initialization
 *   Đây là function có thể được gọi từ board bringup sequence
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_STM32_GPIO_CONTROL_BOARD_INIT
int board_gpio_initialize(void)
{
  return board_gpio_setup();
}
#endif  