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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stm32_spidev_initialize(void)
{
  struct spi_dev_s *spi;

  /* Configure SPI1 pins */
#ifdef CONFIG_STM32H7_SPI1