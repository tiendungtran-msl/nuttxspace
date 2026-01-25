#include <nuttx/config.h>
#include <errno.h>
#include <syslog.h>

int weact_gpio_init(void);
int weact_spi_init(void);

int stm32_bringup(void)
{
  int ret;

  /* 1) Safe-state GPIO first (CS inactive, rails stable) */
  weact_gpio_init();
uint32_t board_userled_initialize(void)


  /* 2) Init SPI buses */
#ifdef CONFIG_SPI
  ret = weact_spi_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "weact: SPI init failed: %d\n", ret);
      /* Continue or return error depending on your policy */
      return ret;
    }
#endif

  /* 3) Future: init I2C/UART/SDMMC mount here */

  return OK;
}