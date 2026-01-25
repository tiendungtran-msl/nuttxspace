/****************************************************************************
 * boards/arm/stm32h7/weact-stm32h743/src/weact_bringup.c
 ****************************************************************************/

#include <nuttx/config.h>
#include <syslog.h>
#include <errno.h>

int weact_spi_init(void);
int weact_board_serial_init(void);
void weact_board_gpio_init(void);

int weact_bringup(void)
{
  int ret;

  /* 1) Safe GPIO state first (CS high, LED off) */
  weact_board_gpio_init();

#ifdef CONFIG_USERLED
  /* Initializes user LEDs framework (also configures PC0) */
  extern uint32_t board_userled_initialize(void);
  board_userled_initialize();
#endif

  /* 2) SPI bus init */
  ret = weact_board_spi_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "SPI init failed: %d\n", ret);
      /* Continue booting is sometimes useful, but for UAV you may return ret */
    }

  /* 3) Serial init placeholder */
  ret = weact_board_serial_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Serial init failed: %d\n", ret);
    }

  return OK;
}