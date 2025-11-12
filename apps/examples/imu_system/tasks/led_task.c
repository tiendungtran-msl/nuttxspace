/****************************************************************************
 * apps/examples/imu_system/tasks/led_task.c
 *
 * LED Status Indication Task
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include <debug.h>

#include "tasks.h"
#include "../drivers/led/led_driver.h"
#include "../utils/data_queue.h"
#include "../utils/config.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

extern data_queue_t g_sensor_queue;
extern data_queue_t g_fusion_queue;
extern system_state_t g_system_state;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_task_main
 ****************************************************************************/

void *led_task_main(void *arg)
{
  uint64_t period_us = 1000000 / IMU_LED_RATE_HZ;
  uint64_t next_time;
  uint64_t current_time;
  size_t sensor_count;
  size_t fusion_count;

  sninfo("LED task started\n");

  /* Get initial time */

  next_time = clock_systime_ticks() * (1000000 / CLK_TCK);

  while (g_system_state.running)
    {
      /* Get queue status */

      sensor_count = data_queue_count(&g_sensor_queue);
      fusion_count = data_queue_count(&g_fusion_queue);

      /* Determine LED pattern based on system state */

      if (g_system_state.calibration_mode)
        {
          /* Slow blink during calibration */

          led_blink(IMU_LED_SLOW_BLINK_MS);
        }
      else if (sensor_count >= IMU_SENSOR_QUEUE_SIZE - 10)
        {
          /* Fast blink if sensor queue nearly full (error condition) */

          led_blink(IMU_LED_FAST_BLINK_MS);
        }
      else if (sensor_count > 0 && fusion_count > 0)
        {
          /* Solid on during normal operation */

          led_set(true);
        }
      else
        {
          /* Off if no data flowing */

          led_set(false);
        }

      /* Update LED state (handles blinking) */

      led_update();

      /* Wait for next period */

      next_time += period_us;
      current_time = clock_systime_ticks() * (1000000 / CLK_TCK);

      if (next_time > current_time)
        {
          usleep(next_time - current_time);
        }
      else
        {
          next_time = current_time;
        }
    }

  /* Turn off LED on exit */

  led_set(false);

  sninfo("LED task stopped\n");
  return NULL;
}
