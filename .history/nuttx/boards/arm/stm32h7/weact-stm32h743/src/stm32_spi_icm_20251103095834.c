/****************************************************************************
 * boards/arm/stm32h7/weact-stm32h743/src/stm32_spi_icm.c
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO Configuration for ICM42688P CS pins */

#define GPIO_SPI1_CS_ICM0    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                              GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN4)

#define GPIO_SPI1_CS_ICM1    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                              GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN11)

#define GPIO_SPI1_CS_ICM2    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                              GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN12)

#define GPIO_SPI1_CS_ICM3    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                              GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN13)

/* SPI Configuration for ICM42688P */

#define ICM42688P_SPI_FREQUENCY     10000000  /* 10 MHz */
#define ICM42688P_SPI_MODE          SPIDEV_MODE3

/* Number of ICM42688P sensors */

#define NUM_ICM42688P               4

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SPI1 device handle */

static struct spi_dev_s *g_spi1_icm = NULL;

/* CS GPIO pins array */

static const uint32_t g_icm_cs_pins[NUM_ICM42688P] =
{
  GPIO_SPI1_CS_ICM0,
  GPIO_SPI1_CS_ICM1,
  GPIO_SPI1_CS_ICM2,
  GPIO_SPI1_CS_ICM3
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_icm_initialize
 *
 * Description:
 *   Initialize CS GPIO pins for ICM42688P sensors
 *
 ****************************************************************************/

void stm32_spidev_icm_initialize(void)
{
  int i;

  _info("Configuring CS pins for %d ICM42688P sensors\n", NUM_ICM42688P);

  /* Configure all CS pins as GPIO outputs, default HIGH (inactive) */

  for (i = 0; i < NUM_ICM42688P; i++)
    {
      int ret = stm32_configgpio(g_icm_cs_pins[i]);
      if (ret < 0)
        {
          _err("ERROR: Failed to configure CS pin %d (GPIO 0x%08lx)\n", 
               i, (unsigned long)g_icm_cs_pins[i]);
          continue;
        }

      stm32_gpiowrite(g_icm_cs_pins[i], 1);  /* CS idle HIGH */
      _info("  ICM%d CS pin configured OK\n", i);
    }

  _info("ICM42688P CS pins initialization completed\n");
}

/****************************************************************************
 * Name: board_spi1_icm_initialize
 *
 * Description:
 *   Initialize SPI1 bus for ICM42688P communication
 *
 ****************************************************************************/

int board_spi1_icm_initialize(void)
{
  uint32_t actual_freq;

  _info("Starting SPI1 initialization for ICM42688P\n");

  /* Get SPI1 bus instance */

  g_spi1_icm = stm32_spibus_initialize(1);
  if (g_spi1_icm == NULL)
    {
      _err("ERROR: Failed to initialize SPI1 bus\n");
      return -ENODEV;
    }

  _info("SPI1 bus handle obtained\n");

  /* Configure SPI1 parameters for ICM42688P */

  SPI_LOCK(g_spi1_icm, true);

  /* Set SPI Mode 3 (CPOL=1, CPHA=1) */

  SPI_SETMODE(g_spi1_icm, ICM42688P_SPI_MODE);
  _info("  Mode set to: 3\n");

  /* Set 8-bit data width */

  SPI_SETBITS(g_spi1_icm, 8);
  _info("  Data bits set to: 8\n");

  /* Set frequency to 10 MHz */

  actual_freq = SPI_SETFREQUENCY(g_spi1_icm, ICM42688P_SPI_FREQUENCY);
  _info("  Frequency set to: %lu Hz (requested: %d Hz)\n", 
        (unsigned long)actual_freq, ICM42688P_SPI_FREQUENCY);

  SPI_LOCK(g_spi1_icm, false);

  _info("SPI1 configuration completed successfully\n");

  return OK;
}

/****************************************************************************
 * Name: board_spi1_icm_select
 *
 * Description:
 *   Select/deselect ICM42688P chip select
 *
 ****************************************************************************/

void board_spi1_icm_select(uint8_t id, bool selected)
{
  if (id < NUM_ICM42688P)
    {
      stm32_gpiowrite(g_icm_cs_pins[id], !selected);
    }
}

/****************************************************************************
 * Name: board_spi1_icm_get_handle
 *
 * Description:
 *   Get SPI1 device handle for ICM42688P
 *
 ****************************************************************************/

struct spi_dev_s *board_spi1_icm_get_handle(void)
{
  return g_spi1_icm;
}