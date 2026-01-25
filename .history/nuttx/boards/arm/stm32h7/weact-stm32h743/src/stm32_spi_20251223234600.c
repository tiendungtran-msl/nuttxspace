#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <errno.h>
#include <syslog.h>

#include "board.h"

/* Keep a single SPI1 instance for the whole system */
static FAR struct spi_dev_s *g_spi1;

FAR struct spi_dev_s *weact_spi1(void)
{
  return g_spi1;
}

int weact_spi_init(void)
{
#ifdef CONFIG_STM32H7_SPI1
  g_spi1 = stm32_spibus_initialize(1);
  if (g_spi1 == NULL)
    {
      syslog(LOG_ERR, "weact: Failed to initialize SPI1\n");
      return -ENODEV;
    }

  /* Optional: set a conservative default.
   * Driver will set exact freq/mode as needed before transactions.
   */
  SPI_SETBITS(g_spi1, 8);
  SPI_SETFREQUENCY(g_spi1, 1000000);

  /* NOTE: Do NOT force mode here if you expect multiple devices with
   * different modes on the same bus. Let each driver set mode before use.
   */

  return OK;
#else
  return -ENOSYS;
#endif
}

/* Board-specific chip select routing for SPI1.
 * NuttX core calls this via SPI_SELECT() with a devid.
 */
void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  /* Active-low chip select */
  const bool inactive = !selected;

  switch (devid)
    {
      case SPIDEV_IMU0:
        stm32_gpiowrite(GPIO_IMU0_CS, inactive);
        break;

      case SPIDEV_IMU1:
        stm32_gpiowrite(GPIO_IMU1_CS, inactive);
        break;

      case SPIDEV_IMU2:
        stm32_gpiowrite(GPIO_IMU2_CS, inactive);
        break;

      case SPIDEV_IMU3:
        stm32_gpiowrite(GPIO_IMU3_CS, inactive);
        break;

      default:
        /* Unknown device id: leave all CS inactive */
        break;
    }
}

/* Optional presence/status hook */
uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  (void)dev;
  (void)devid;
  return SPI_STATUS_PRESENT;
}