/****************************************************************************
 * apps/examples/uav_states/tests/icm42688p_test.c
 *
 *   ICM-42688-P Simple Test Program (Fixed & Enhanced version)
 *   Compatible with NuttX + uav_states ICM42688P driver
 *
 *   Licensed under Apache-2.0
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <inttypes.h>

#include "../drivers/sensors/icm42688p/icm42688p_driver.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_UAV_STATES_ICM42688P_SPI_BUS
#  define CONFIG_UAV_STATES_ICM42688P_SPI_BUS 1
#endif

#ifndef CONFIG_UAV_STATES_ICM42688P_DEVID
#  define CONFIG_UAV_STATES_ICM42688P_DEVID 0
#endif

#define DEFAULT_SAMPLE_COUNT    100
#define DEFAULT_SAMPLE_RATE_HZ  1000    /* ICM-42688-P hỗ trợ tốt đến 8kHz (accel) / 32kHz (gyro) */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile sig_atomic_t g_test_running = true;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void signal_handler(int signo)
{
  (void)signo;
  g_test_running = false;
}

static void print_usage(const char *progname)
{
  printf("\nICM-42688-P Sensor Test Program\n");
  printf("Usage: %s [OPTIONS]\n\n", progname);
  printf("Options:\n");
  printf("  -c             Calibrate gyroscope (keep device stationary)\n");
  printf("  -n <count>     Number of samples (default: %d, 0 = continuous)\n",
         DEFAULT_SAMPLE_COUNT);
  printf("  -r <rate>      Sample rate in Hz (1 - 8000, default: %d)\n",
         DEFAULT_SAMPLE_RATE_HZ);
  printf("  -h             Show this help\n");
  printf("\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int icm42688p_test_main(int argc, char *argv[])
{
  icm42688p_dev_t dev = {0};
  icm42688p_data_t data;
  bool do_calibration = false;
  int sample_count = DEFAULT_SAMPLE_COUNT;
  int sample_rate_hz = DEFAULT_SAMPLE_RATE_HZ;
  int opt;
  int ret;
  int samples_read = 0;
  uint64_t start_time_us = 0;

  /* Parse command line arguments */
  while ((opt = getopt(argc, argv, "cn:r:h")) != -1)
    {
      switch (opt)
        {
          case 'c':
            do_calibration = true;
            break;

          case 'n':
            sample_count = atoi(optarg);
            if (sample_count < 0)
              {
                fprintf(stderr, "ERROR: Sample count must be >= 0\n");
                return EXIT_FAILURE;
              }
            break;

          case 'r':
            sample_rate_hz = atoi(optarg);
            if (sample_rate_hz <= 0 || sample_rate_hz > 8000)
              {
                fprintf(stderr, "ERROR: Sample rate must be 1 - 8000 Hz\n");
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

  /* Setup signal handlers */
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  printf("\n");
  printf("======================================\n");
  printf("    ICM-42688-P Sensor Test Program    \n");
  printf("======================================\n\n");

  printf("Initializing ICM-42688-P on SPI%d CS%d...\n",
         CONFIG_UAV_STATES_ICM42688P_SPI_BUS,
         CONFIG_UAV_STATES_ICM42688P_DEVID);

  ret = icm42688p_init(&dev,
                       CONFIG_UAV_STATES_ICM42688P_SPI_BUS,
                       CONFIG_UAV_STATES_ICM42688P_DEVID);
  if (ret != ICM42688P_OK)
    {
      fprintf(stderr, "ERROR: Failed to initialize ICM-42688-P (ret=%d)\n", ret);
      return EXIT_FAILURE;
    }
  printf("ICM-42688-P initialized successfully!\n\n");

  /* Self-test */
  printf("Running hardware self-test...\n");
  ret = icm42688p_self_test(&dev);
  if (ret == ICM42688P_OK)
    printf("Self-test: PASSED\n\n");
  else
    printf("Self-test: FAILED (ret=%d) - continuing anyway...\n\n", ret);

  /* Gyroscope calibration */
  if (do_calibration)
    {
      printf("Calibrating gyroscope... (keep device perfectly still!)\n");
      ret = icm42688p_calibrate_gyro(&dev, 500);  /* 500 samples ~2-3s */
      if (ret == ICM42688P_OK)
        printf("Gyro calibration completed successfully!\n\n");
      else
        printf("Gyro calibration FAILED (ret=%d)\n\n", ret);
    }

  /* Start data acquisition */
  char count_str[64];
  if (sample_count == 0)
    strcpy(count_str, "Continuous (press Ctrl+C to stop)");
  else
    snprintf(count_str, sizeof(count_str), "%d", sample_count);

  printf("Starting data acquisition...\n");
  printf(" Sample rate : %d Hz\n", sample_rate_hz);
  printf(" Sample count: %s\n\n", count_str);

  /* Header */
  printf("   Time(s)   |  Accel X    Y    Z   (g)  |  Gyro X     Y     Z   (dps)  | Temp (°C)\n");
  printf("-------------|----------------------------|------------------------------|----------\n");
  fflush(stdout);

  start_time_us = data.timestamp;  /* Will be set on first read */

  while (g_test_running && (sample_count == 0 || samples_read < sample_count))
    {
      ret = icm42688p_read_data(&dev, &data);
      if (ret != ICM42688P_OK)
        {
          fprintf(stderr, "\rRead error: %d        \n", ret);
          usleep(10000);
          continue;
        }

      /* Use relative time from first sample */
      if (samples_read == 0)
        start_time_us = data.timestamp;

      double time_sec = (data.timestamp - start_time_us) / 1000000.0;

      printf("\r%9.3f | %6.3f %6.3f %6.3f | %7.2f %7.2f %7.2f | %6.2f",
             time_sec,
             data.accel.x,  data.accel.y,  data.accel.z,
             data.gyro.x,   data.gyro.y,   data.gyro.z,
             data.temperature);

      fflush(stdout);
      samples_read++;

      /* Control sample rate */
      int delay_us = 1000000 / sample_rate_hz;
      if (delay_us > 0)
        usleep(delay_us);
    }

  printf("\n\n");

  /* Final statistics */
  printf("======================================\n");
  printf("Test completed!\n");
  printf("Samples read : %d\n",  samples_read);
  if (samples_read > 0)
    {
      double duration = (data.timestamp - start_time_us) / 1000000.0;
      printf("Duration     : %.3f seconds\n", duration);
      printf("Avg rate     : %.1f Hz\n", samples_read / duration);
    }
  printf("======================================\n\n");

  icm42688p_print_status(&dev);
  printf("\n");

  /* Cleanup */
  icm42688p_deinit(&dev);
  printf("Sensor deinitialized. Goodbye!\n");

  return EXIT_SUCCESS;
}