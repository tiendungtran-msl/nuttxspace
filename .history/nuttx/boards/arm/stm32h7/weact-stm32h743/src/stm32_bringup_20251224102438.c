/****************************************************************************
 * boards/arm/stm32h7/weact-stm32h743/src/stm32_bringup. c
 ****************************************************************************/

#include <nuttx/config.h>
#include <syslog.h>

#include "weact-stm32h743.h"

int weact_bringup(void)
{
  int ret;

  /* 1. GPIO safe-state (CS high, LED off) */
  weact_gpio_init();

#ifdef CONFIG_USERLED
  /* Initialize user LED framework */
  board_userled_initialize();
#endif

  /* 2. SPI bus initialization */
  ret = weact_spi_bus_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "SPI bus init failed: %d\n", ret);
    }

  /* 3. UART (placeholder) */
  ret = weact_usart_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "USART init failed: %d\n", ret);
    }

  /* 4. I2C (placeholder for future mag) */
#ifdef CONFIG_STM32H7_I2C1
  ret = weact_i2c_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "I2C init failed: %d\n", ret);
    }
#endif

  /* 5. SDMMC (if needed) - you may already have stm32_sdmmc. c doing this */

  return OK;
}