/****************************************************************************
 * boards/arm/stm32/stm32f411-minimum/src/stm32_appinit.c
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/boardctl.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/task.h>

#include "stm32f411-minimum.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_STM32_BLINK_PRIORITY
#  define CONFIG_EXAMPLES_STM32_BLINK_PRIORITY 100
#endif

#ifndef CONFIG_EXAMPLES_STM32_BLINK_STACKSIZE
#  define CONFIG_EXAMPLES_STM32_BLINK_STACKSIZE 2048
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Entry point for the blink application */

extern int stm32_blink_main(int argc, char *argv[]);

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization. This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
  int ret;

  /* Perform board-specific initialization */

  ret = stm32_bringup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_bringup failed: %d\n", ret);
      return ret;
    }

  /* Auto-start the blink application */

#ifdef CONFIG_EXAMPLES_STM32_BLINK
  ret = task_create("stm32_blink",
                    CONFIG_EXAMPLES_STM32_BLINK_PRIORITY,
                    CONFIG_EXAMPLES_STM32_BLINK_STACKSIZE,
                    stm32_blink_main,
                    NULL);

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to start stm32_blink: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "STM32 Blink application started (PID: %d)\n", ret);
    }
#endif

  return OK;
}