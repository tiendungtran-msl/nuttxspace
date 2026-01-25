/****************************************************************************
 * boards/arm/stm32h7/custom-h743-vectornav/src/stm32_spi. c
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