/****************************************************************************
 * boards/arm/stm32h7/weact-stm32h743/src/stm32_appinitialize.c
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/board.h>

#include "weact-stm32h743.h"

#ifdef CONFIG_BOARDCTL

int board_app_initialize(uintptr_t arg)
{
  return weact_bringup();
}

#endif