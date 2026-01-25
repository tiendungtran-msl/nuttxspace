/****************************************************************************
 * boards/arm/stm32h7/uav_states/src/stm32_spi.c
 *
 * SPI bus initialization and chip select management for multiple devices
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Thiết lập bus SPI và các thiết bị SPI được sử dụng */
void stm32_spidev_initialize(void)
{
  struct spi_dev_s *spi;

  /* Configure SPI1 pins */
#ifdef CONFIG_STM32H7_SPI1
  spi = stm32_spibus_initialize(1);
  if (spi)
    {
      /* Configure CS pins */
      stm32_configgpio(GPIO_SPI1_CS_ICM0);
      stm32_configgpio(GPIO_SPI1_CS_ICM1);
      stm32_configgpio(GPIO_SPI1_CS_ICM2);
      stm32_configgpio(GPIO_SPI1_CS_ICM3);
    }
  else
    {
      spierr("ERROR: Failed to initialize SPI1\n");
    }
#endif
}

#ifdef CONFIG_STM32H7_SPI1
void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  switch (devid)
    {
      case SPIDEV_ICM0:
        stm32_gpiowrite(GPIO_SPI1_CS_ICM0, !selected);
        break;

      case SPIDEV_ICM1:
        stm32_gpiowrite(GPIO_SPI1_CS_ICM1, !selected);
        break;

      case SPIDEV_ICM2:
        stm32_gpiowrite(GPIO_SPI1_CS_ICM2, !selected);
        break;

      case SPIDEV_ICM3:
        stm32_gpiowrite(GPIO_SPI1_CS_ICM3, !selected);
        break;

      default:
        spierr("ERROR: Unrecognized devid: %d\n", devid);
        break;
    }
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_SPI_CMDDATA
int stm32_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  /* This function is not needed for the ICM42688P */
  return OK;
}
#endif
