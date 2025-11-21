/****************************************************************************
 * apps/examples/imu_system/tasks/sensor_task.c
 *
 * Sensor Reading Task
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include <debug.h>
#include <nuttx/clock.h>

#include "tasks.h"
#include "../drivers/spi/spi_manager.h"
#include "../utils/data_queue.h"
#include "../utils/config.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

extern data_queue_t g_sensor_queue;
extern system_state_t g_system_state;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sensor_task_main
 ****************************************************************************/

void *sensor_task_main(void *arg)
{
  sensor_data_packet_t packet;
  uint64_t period_us = 1000000 / IMU_SENSOR_RATE_HZ;
  uint64_t next_time;
  uint64_t current_time;
  int i;
  int ret;

  sninfo("Sensor task started\n");

  /* Get initial time */

  next_time = clock_systime_ticks() * (1000000 / CLK_TCK);

  while (g_system_state.running)
    {
      /* Lock SPI bus */

      ret = spi_manager_lock();
      if (ret < 0)
        {
          snerr("ERROR: Failed to lock SPI bus\n");
          usleep(1000);
          continue;
        }

      /* Read all 4 ICM42688P sensors */

      for (i = 0; i < IMU_NUM_ICM42688P; i++)
        {
          ret = icm42688p_read(i, &packet.imu[i]);
          if (ret < 0)
            {
              snerr("ERROR: Failed to read ICM42688P[%d]\n", i);
            }
        }

      /* Read BMM150 magnetometer */

      ret = bmm150_read(&packet.mag);
      if (ret < 0)
        {
          snerr("ERROR: Failed to read BMM150\n");
        }

      /* Unlock SPI bus */

      spi_manager_unlock();

      /* Set timestamp */

      packet.timestamp = clock_systime_ticks() * (1000000 / CLK_TCK);

      /* Push to queue */

      ret = data_queue_push(&g_sensor_queue, &packet);
      if (ret < 0)
        {
          /* Queue full - data dropped */

          static uint32_t drop_count = 0;
          if ((++drop_count % 100) == 0)
            {
              snerr("WARNING: Sensor queue full, %u samples dropped\n",
                    drop_count);
            }
        }

      /* Wait for next period */

      next_time += period_us;
      current_time = clock_systime_ticks() * (1000000 / CLK_TCK);

      if (next_time > current_time)
        {
          usleep(next_time - current_time);
        }
      else
        {
          /* Missed deadline */

          static uint32_t miss_count = 0;
          if ((++miss_count % 100) == 0)
            {
              snerr("WARNING: Sensor task missed %u deadlines\n",
                    miss_count);
            }

          next_time = current_time;
        }
    }

  sninfo("Sensor task stopped\n");
  return NULL;
}
