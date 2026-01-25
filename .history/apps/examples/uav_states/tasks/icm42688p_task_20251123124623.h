#include "icm42688p_task.h"
/****************************************************************************
 * apps/examples/uav_states/tests/icm42688p_test.c
 ****************************************************************************/ 
#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include <debug.h>
#include <errno.h>
#include <syslog.h>
#include <signal.h>
#include <stdbool.h>
#include <nuttx/clock.h>
#include "../drivers/sensors/icm42688p/icm42688p_driver.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
