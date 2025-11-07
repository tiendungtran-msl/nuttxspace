/****************************************************************************
 * boards/arm/stm32h7/your-board/src/stm32_spi_icm.c
 *
 * Description:
 *   SPI1 peripheral initialization for 4x ICM42688P IMU sensors
 *   on STM32H743VIT6
 *
 * Author: tiendungtran-msl
 * Date: 2025-11-03
 ****************************************************************************/

/****************************************************************************
 * Included Files
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
 *   Called from stm32_spidev_initialize() before SPI bus initialization
 *
 ****************************************************************************/

void stm32_spidev_icm_initialize(void)
{
  int i;

  spiinfo("Configuring CS pins for %d ICM42688P sensors\n", NUM_ICM42688P);

  /* Configure all CS pins as GPIO outputs, default HIGH (inactive) */

  for (i = 0; i < NUM_ICM42688P; i++)
    {
      stm32_configgpio(g_icm_cs_pins[i]);
      stm32_gpiowrite(g_icm_cs_pins[i], 1);  /* CS idle HIGH */
      spiinfo("  ICM%d CS pin configured (GPIO 0x%08lx)\n", i, (unsigned long)g_icm_cs_pins[i]);
    }

  spiinfo("ICM42688P CS pins initialization completed\n");
}

/****************************************************************************
 * Name: board_spi1_icm_initialize
 *
 * Description:
 *   Initialize SPI1 bus for ICM42688P communication
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure
 *
 ****************************************************************************/

int board_spi1_icm_initialize(void)
{
  spiinfo("Initializing SPI1 for ICM42688P sensors\n");

  /* Get SPI1 bus instance */

  g_spi1_icm = stm32_spibus_initialize(1);
  if (g_spi1_icm == NULL)
    {
      spierr("ERROR: Failed to initialize SPI1 bus\n");
      return -ENODEV;
    }

  spiinfo("SPI1 bus initialized successfully\n");

  /* Configure SPI1 parameters for ICM42688P */

  SPI_LOCK(g_spi1_icm, true);

  /* Set SPI Mode 3 (CPOL=1, CPHA=1) */

  SPI_SETMODE(g_spi1_icm, ICM42688P_SPI_MODE);
  spiinfo("  Mode: 3 (CPOL=1, CPHA=1)\n");

  /* Set 8-bit data width */

  SPI_SETBITS(g_spi1_icm, 8);
  spiinfo("  Data bits: 8\n");

  /* Set frequency to 10 MHz */

  SPI_SETFREQUENCY(g_spi1_icm, ICM42688P_SPI_FREQUENCY);
  spiinfo("  Frequency: %d Hz\n", ICM42688P_SPI_FREQUENCY);

  SPI_LOCK(g_spi1_icm, false);

  spiinfo("SPI1 configuration completed for ICM42688P\n");

  return OK;
}

/****************************************************************************
 * Name: board_spi1_icm_select
 *
 * Description:
 *   Select/deselect ICM42688P chip select
 *
 * Input Parameters:
 *   id       - Sensor ID (0-3)
 *   selected - true: assert CS (LOW), false: deassert CS (HIGH)
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
 * Returned Value:
 *   Pointer to SPI device structure, or NULL if not initialized
 *
 ****************************************************************************/

struct spi_dev_s *board_spi1_icm_get_handle(void)
{
  return g_spi1_icm;
}