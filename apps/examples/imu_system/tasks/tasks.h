/****************************************************************************
 * apps/examples/imu_system/tasks/tasks.h
 *
 * Task Definitions and Coordination
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_IMU_SYSTEM_TASKS_TASKS_H
#define __APPS_EXAMPLES_IMU_SYSTEM_TASKS_TASKS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#include "../drivers/icm42688p/icm42688p_driver.h"
#include "../drivers/bmm150/bmm150_driver.h"
#include "../algorithms/sensor_fusion.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Sensor data packet */

typedef struct
{
  icm42688p_data_t imu[4];  /* 4 IMU sensors */
  bmm150_data_t mag;        /* Magnetometer */
  uint64_t timestamp;
} sensor_data_packet_t;

/* System state */

typedef struct
{
  bool running;
  bool calibration_mode;
  pthread_t sensor_thread;
  pthread_t fusion_thread;
  pthread_t led_thread;
  pthread_mutex_t mutex;
} system_state_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sensor_task_main
 *
 * Description:
 *   Sensor reading task (1kHz)
 *
 ****************************************************************************/

void *sensor_task_main(void *arg);

/****************************************************************************
 * Name: fusion_task_main
 *
 * Description:
 *   Sensor fusion task (100Hz)
 *
 ****************************************************************************/

void *fusion_task_main(void *arg);

/****************************************************************************
 * Name: led_task_main
 *
 * Description:
 *   LED status task (10Hz)
 *
 ****************************************************************************/

void *led_task_main(void *arg);

#endif /* __APPS_EXAMPLES_IMU_SYSTEM_TASKS_TASKS_H */
