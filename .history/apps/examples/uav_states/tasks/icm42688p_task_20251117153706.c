/****************************************************************************
 * apps/examples/uav_states/tasks/icm42688p_task.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <errno.h>
#include <debug.h>

#include "../drivers/sensors/icm42688p/icm42688p_driver.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_UAV_STATES_ICM42688P_SPI_BUS
#  define CONFIG_UAV_STATES_ICM42688P_SPI_BUS       1
#endif

#ifndef CONFIG_UAV_STATES_ICM42688P_DEVID
#  define CONFIG_UAV_STATES_ICM42688P_DEVID         0
#endif

#ifndef CONFIG_UAV_STATES_ICM42688P_FREQUENCY
#  define CONFIG_UAV_STATES_ICM42688P_FREQUENCY     8000000
#endif

#define ICM42688P_TASK_SAMPLE_RATE_HZ   100   /* 100 Hz */
#define ICM42688P_TASK_DELAY_US         (1000000 / ICM42688P_TASK_SAMPLE_RATE_HZ)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static icm42688p_dev_t g_icm42688p_dev;
static volatile bool g_icm42688p_task_running = false;
static pthread_t g_icm42688p_thread;
static icm42688p_data_t g_latest_data;
static pthread_mutex_t g_data_mutex = PTHREAD_MUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icm42688p_task_thread
 *
 * Description:
 *   Main thread function for ICM42688P data acquisition
 *
 ****************************************************************************/

static void *icm42688p_task_thread(void *arg)
{
  icm42688p_data_t data;
  int ret;
  uint32_t error_count = 0;

  _info("ICM42688P task started\n");

  while (g_icm42688p_task_running)
    {
      /* Read sensor data */

      ret = icm42688p_read_data(&g_icm42688p_dev, &data);
      if (ret == ICM42688P_OK)
        {
          /* Update shared data with mutex protection */

          pthread_mutex_lock(&g_data_mutex);
          memcpy(&g_latest_data, &data, sizeof(icm42688p_data_t));
          pthread_mutex_unlock(&g_data_mutex);

          /* Reset error count on successful read */

          error_count = 0;
        }
      else
        {
          error_count++;
          _err("ERROR: Failed to read ICM42688P: %d (errors: %u)\n", 
               ret, error_count);

          /* If too many errors, try to reinitialize */

          if (error_count > 10)
            {
              _warn("WARNING: Too many errors, attempting reinit...\n");
              
              icm42688p_deinit(&g_icm42688p_dev);
              usleep(100000);  /* 100ms delay */
              
              ret = icm42688p_init(&g_icm42688p_dev, 
                                   CONFIG_UAV_STATES_ICM42688P_SPI_BUS,
                                   CONFIG_UAV_STATES_ICM42688P_DEVID,
                                   0);  /* SPI Mode 0 */
              
              if (ret == ICM42688P_OK)
                {
                  _info("ICM42688P reinitialized successfully\n");
                  error_count = 0;
                }
            }
        }

      /* Sleep to maintain sample rate */

      usleep(ICM42688P_TASK_DELAY_US);
    }

  _info("ICM42688P task stopped\n");
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icm42688p_task_start
 *
 * Description:
 *   Start the ICM42688P reading task
 *
 * Returned Value:
 *   0 on success; negative errno on failure
 *
 ****************************************************************************/

int icm42688p_task_start(void)
{
  int ret;

  if (g_icm42688p_task_running)
    {
      _warn("WARNING: ICM42688P task already running\n");
      return -EBUSY;
    }

  /* Initialize ICM42688P sensor */

  _info("Initializing ICM42688P on SPI%d, CS%d...\n",
        CONFIG_UAV_STATES_ICM42688P_SPI_BUS,
        CONFIG_UAV_STATES_ICM42688P_DEVID);

  ret = icm42688p_init(&g_icm42688p_dev,
                       CONFIG_UAV_STATES_ICM42688P_SPI_BUS,
                       CONFIG_UAV_STATES_ICM42688P_DEVID,
                       CONFIG_UAV_STATES_ICM42688P_FREQUENCY,
                       0);  /* SPI Mode 0 */

  if (ret != ICM42688P_OK)
    {
      _err("ERROR: Failed to initialize ICM42688P: %d\n", ret);
      return ret;
    }

  _info("ICM42688P initialized successfully\n");

  /* Perform gyroscope calibration */

  _info("Calibrating gyroscope (keep device stationary)...\n");
  ret = icm42688p_calibrate_gyro(&g_icm42688p_dev, 200);
  if (ret != ICM42688P_OK)
    {
      _warn("WARNING: Gyro calibration failed: %d\n", ret);
      /* Continue anyway */
    }
  else
    {
      _info("Gyro calibration complete\n");
    }

  /* Initialize mutex */

  pthread_mutex_init(&g_data_mutex, NULL);

  /* Start task thread */

  g_icm42688p_task_running = true;

  ret = pthread_create(&g_icm42688p_thread, NULL, 
                       icm42688p_task_thread, NULL);
  if (ret != 0)
    {
      _err("ERROR: Failed to create ICM42688P thread: %d\n", ret);
      g_icm42688p_task_running = false;
      icm42688p_deinit(&g_icm42688p_dev);
      return -ret;
    }

  /* Set thread name for debugging */

#ifdef CONFIG_PTHREAD_SETNAME_NP
  pthread_setname_np(g_icm42688p_thread, "icm42688p");
#endif

  _info("ICM42688P task started successfully\n");

  return OK;
}

/****************************************************************************
 * Name: icm42688p_task_stop
 *
 * Description:
 *   Stop the ICM42688P reading task
 *
 * Returned Value:
 *   0 on success; negative errno on failure
 *
 ****************************************************************************/

int icm42688p_task_stop(void)
{
  void *ret_val;

  if (!g_icm42688p_task_running)
    {
      _warn("WARNING: ICM42688P task not running\n");
      return -EINVAL;
    }

  _info("Stopping ICM42688P task...\n");

  /* Signal thread to stop */

  g_icm42688p_task_running = false;

  /* Wait for thread to finish */

  pthread_join(g_icm42688p_thread, &ret_val);

  /* Cleanup */

  icm42688p_deinit(&g_icm42688p_dev);
  pthread_mutex_destroy(&g_data_mutex);

  _info("ICM42688P task stopped\n");

  return OK;
}

/****************************************************************************
 * Name: icm42688p_task_get_data
 *
 * Description:
 *   Get the latest IMU data (thread-safe)
 *
 * Input Parameters:
 *   data - Pointer to store the data
 *
 * Returned Value:
 *   0 on success; negative errno on failure
 *
 ****************************************************************************/

int icm42688p_task_get_data(icm42688p_data_t *data)
{
  if (!data)
    {
      return -EINVAL;
    }

  if (!g_icm42688p_task_running)
    {
      return -ENODEV;
    }

  /* Copy data with mutex protection */

  pthread_mutex_lock(&g_data_mutex);
  memcpy(data, &g_latest_data, sizeof(icm42688p_data_t));
  pthread_mutex_unlock(&g_data_mutex);

  return OK;
}

/****************************************************************************
 * Name: icm42688p_task_is_running
 *
 * Description:
 *   Check if ICM42688P task is running
 *
 * Returned Value:
 *   true if running; false otherwise
 *
 ****************************************************************************/

bool icm42688p_task_is_running(void)
{
  return g_icm42688p_task_running;
}

/****************************************************************************
 * Name: icm42688p_task_print_status
 *
 * Description:
 *   Print ICM42688P task status
 *
 ****************************************************************************/

void icm42688p_task_print_status(void)
{
  icm42688p_data_t data;

  printf("===== ICM42688P Task Status =====\n");
  printf("Running:       %s\n", g_icm42688p_task_running ? "YES" : "NO");
  printf("Sample Rate:   %d Hz\n", ICM42688P_TASK_SAMPLE_RATE_HZ);
  
  if (g_icm42688p_task_running)
    {
      icm42688p_print_status(&g_icm42688p_dev);
      
      if (icm42688p_task_get_data(&data) == OK)
        {
          printf("\n--- Latest Data ---\n");
          printf("Time:  %.3f s\n", data.timestamp / 1000000.0);
          printf("Accel: X=%6.3f Y=%6.3f Z=%6.3f g\n",
                 data.accel.x, data.accel.y, data.accel.z);
          printf("Gyro:  X=%7.2f Y=%7.2f Z=%7.2f dps\n",
                 data.gyro.x, data.gyro.y, data.gyro.z);
          printf("Temp:  %.2f °C\n", data.temperature);
        }
    }
  
  printf("=================================\n");
}