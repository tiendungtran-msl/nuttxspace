/****************************************************************************
 * apps/examples/imu_system/tasks/fusion_task.c
 *
 * Sensor Fusion Task
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include <debug.h>
#include <math.h>
#include <nuttx/clock.h>

#include "tasks.h"
#include "../algorithms/sensor_fusion.h"
#include "../algorithms/orientation.h"
#include "../utils/data_queue.h"
#include "../utils/config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef M_PI
#  define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (M_PI / 180.0f)

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
 * Name: fusion_task_main
 ****************************************************************************/

void *fusion_task_main(void *arg)
{
  sensor_data_packet_t sensor_packet;
  fusion_result_t fusion_result;
  uint64_t period_us = 1000000 / IMU_FUSION_RATE_HZ;
  uint64_t next_time;
  uint64_t current_time;
  float avg_ax;
  float avg_ay;
  float avg_az;
  float avg_gx;
  float avg_gy;
  float avg_gz;
  int i;
  int ret;

  sninfo("Fusion task started\n");

  /* Initialize fusion algorithms */

  fusion_init(IMU_MADGWICK_BETA, IMU_MADGWICK_SAMPLE_FREQ);
  orientation_init();

  /* Get initial time */

  next_time = clock_systime_ticks() * (1000000 / CLK_TCK);

  while (g_system_state.running)
    {
      /* Pop sensor data from queue */

      ret = data_queue_pop(&g_sensor_queue, &sensor_packet);
      if (ret < 0)
        {
          /* No data available, wait a bit */

          usleep(1000);
          continue;
        }

      /* Average data from all 4 IMUs */

      avg_ax = 0.0f;
      avg_ay = 0.0f;
      avg_az = 0.0f;
      avg_gx = 0.0f;
      avg_gy = 0.0f;
      avg_gz = 0.0f;

      for (i = 0; i < IMU_NUM_ICM42688P; i++)
        {
          avg_ax += sensor_packet.imu[i].accel.x;
          avg_ay += sensor_packet.imu[i].accel.y;
          avg_az += sensor_packet.imu[i].accel.z;
          avg_gx += sensor_packet.imu[i].gyro.x;
          avg_gy += sensor_packet.imu[i].gyro.y;
          avg_gz += sensor_packet.imu[i].gyro.z;
        }

      avg_ax /= IMU_NUM_ICM42688P;
      avg_ay /= IMU_NUM_ICM42688P;
      avg_az /= IMU_NUM_ICM42688P;
      avg_gx /= IMU_NUM_ICM42688P;
      avg_gy /= IMU_NUM_ICM42688P;
      avg_gz /= IMU_NUM_ICM42688P;

      /* Convert gyro from dps to rad/s */

      avg_gx *= DEG_TO_RAD;
      avg_gy *= DEG_TO_RAD;
      avg_gz *= DEG_TO_RAD;

      /* Update fusion with averaged IMU data and magnetometer */

      fusion_update(avg_gx, avg_gy, avg_gz,
                    avg_ax, avg_ay, avg_az,
                    sensor_packet.mag.x,
                    sensor_packet.mag.y,
                    sensor_packet.mag.z);

      /* Get fusion result */

      fusion_get_result(&fusion_result);

      /* Calculate orientation/heading */

      orientation_calculate(fusion_result.euler.roll,
                           fusion_result.euler.pitch,
                           sensor_packet.mag.x,
                           sensor_packet.mag.y,
                           sensor_packet.mag.z);

      /* Push result to fusion queue */

      ret = data_queue_push(&g_fusion_queue, &fusion_result);
      if (ret < 0)
        {
          /* Queue full - not critical */
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
          next_time = current_time;
        }
    }

  sninfo("Fusion task stopped\n");
  return NULL;
}
