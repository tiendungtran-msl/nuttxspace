/****************************************************************************
 * apps/examples/uav_states/tests/icm42688p_test.c
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
#include <signal.h>

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

#define TEST_SAMPLE_COUNT   100
#define TEST_SAMPLE_RATE_HZ 100

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile bool g_test_running = true;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: signal_handler
 ****************************************************************************/

static void signal_handler(int signo)
{
  g_test_running = false;
}

/****************************************************************************
 * Name: print_usage
 ****************************************************************************/

static void print_usage(const char *progname)
{
  printf("Usage: %s [OPTIONS]\n", progname);
  printf("Simple ICM42688P sensor test\n\n");
  printf("Options:\n");
  printf("  -c          Calibrate gyroscope before test\n");
  printf("  -n <count>  Number of samples (default: %d, 0=continuous)\n", 
         TEST_SAMPLE_COUNT);
  printf("  -r <rate>   Sample rate in Hz (default: %d)\n", 
         TEST_SAMPLE_RATE_HZ);
  printf("  -h          Show this help\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icm42688p_test_main
 ****************************************************************************/

int icm42688p_test_main(int argc, char *argv[])
{
  icm42688p_dev_t dev;
  icm42688p_data_t data;
  bool do_calibration = false;
  int sample_count = TEST_SAMPLE_COUNT;
  int sample_rate_hz = TEST_SAMPLE_RATE_HZ;
  int opt;
  int ret;
  int samples_read = 0;

  /* Parse command line arguments */

  while ((opt = getopt(argc, argv, "cn:r:h")) != ERROR)
    {
      switch (opt)
        {
          case 'c':
            do_calibration = true;
            break;

          case 'n':
            sample_count = atoi(optarg);
            break;

          case 'r':
            sample_rate_hz = atoi(optarg);
            if (sample_rate_hz <= 0 || sample_rate_hz > 1000)
              {
                fprintf(stderr, "ERROR: Invalid sample rate\n");
                return EXIT_FAILURE;
              }
            break;

          case 'h':
            print_usage(argv[0]);
            return EXIT_SUCCESS;

          default:
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

  /* Setup signal handler */

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  printf("\n======================================\n");
  printf("   ICM42688P Simple Test\n");
  printf("======================================\n\n");

  /* Initialize sensor */

  printf("Initializing ICM42688P...\n");
  printf("  SPI Bus:      %d\n", CONFIG_UAV_STATES_ICM42688P_SPI_BUS);
  printf("  Device ID:    %d\n", CONFIG_UAV_STATES_ICM42688P_DEVID);
  printf("  SPI Freq:     %d Hz\n", CONFIG_UAV_STATES_ICM42688P_FREQUENCY);
  printf("\n");

  ret = icm42688p_init(&dev,
                       CONFIG_UAV_STATES_ICM42688P_SPI_BUS,
                       CONFIG_UAV_STATES_ICM42688P_DEVID);

  if (ret != ICM42688P_OK)
    {
      fprintf(stderr, "ERROR: Failed to initialize ICM42688P: %d\n", ret);
      return EXIT_FAILURE;
    }

  printf("ICM42688P initialized successfully!\n\n");

  /* Self-test */

  printf("Running self-test...\n");
  ret = icm42688p_self_test(&dev);
  if (ret != ICM42688P_OK)
    {
      fprintf(stderr, "WARNING: Self-test failed: %d\n", ret);
    }
  else
    {
      printf("Self-test PASSED\n");
    }
  printf("\n");

  /* Calibrate gyroscope if requested */

  if (do_calibration)
    {
      printf("Calibrating gyroscope...\n");
      printf("(Keep device stationary!)\n");
      
      ret = icm42688p_calibrate_gyro(&dev, 200);
      if (ret != ICM42688P_OK)
        {
          fprintf(stderr, "WARNING: Calibration failed: %d\n", ret);
        }
      else
        {
          printf("Calibration complete!\n");
        }
      printf("\n");
    }

  /* Read and display data */

  printf("Starting data acquisition...\n");
  printf("  Sample Rate:  %d Hz\n", sample_rate_hz);
  printf("  Sample Count: %s\n", 
         sample_count == 0 ? "Continuous (Ctrl+C to stop)" : 
         (char[20]){0} + sprintf((char[20]){0}, "%d", sample_count));
  printf("\n");

  printf("Time(s) | Accel X    Y    Z (g)   | Gyro X     Y     Z (dps)  | Temp(°C)\n");
  printf("--------|-------------------------|---------------------------|----------\n");

  /* Main loop */

  while (g_test_running && (sample_count == 0 || samples_read < sample_count))
    {
      ret = icm42688p_read_data(&dev, &data);
      if (ret != ICM42688P_OK)
        {
          fprintf(stderr, "ERROR: Read failed: %d\n", ret);
          usleep(100000);
          continue;
        }

      /* Print data */

      printf("%6.3f  | %6.3f %6.3f %6.3f | %7.2f %7.2f %7.2f | %6.2f\n",
             data.timestamp / 1000000.0,
             data.accel.x, data.accel.y, data.accel.z,
             data.gyro.x, data.gyro.y, data.gyro.z,
             data.temperature);

      samples_read++;

      /* Sleep to maintain sample rate */

      usleep(1000000 / sample_rate_hz);
    }

  printf("\n");
  printf("======================================\n");
  printf("Total samples read: %d\n", samples_read);
  printf("======================================\n\n");

  /* Print final status */

  icm42688p_print_status(&dev);

  /* Cleanup */

  icm42688p_deinit(&dev);
  printf("\nTest completed\n");

  return EXIT_SUCCESS;
}