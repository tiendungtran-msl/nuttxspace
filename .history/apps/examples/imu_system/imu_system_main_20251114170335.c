/****************************************************************************
 * apps/examples/imu_system/imu_system_main.c
 *
 * IMU System Main Entry Point
 * Hardware: 4x ICM42688P (SPI) + 1x BMM150 (I2C)
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <debug.h>
#include <pthread.h>
#include <math.h>

#include "drivers/spi/spi_manager.h"
#include "drivers/i2c/i2c_manager.h"
#include "drivers/icm42688p/icm42688p_driver.h"
#include "drivers/bmm150/bmm150_driver.h"
#include "drivers/led/led_driver.h"
#include "algorithms/sensor_fusion.h"
#include "algorithms/orientation.h"
#include "tasks/tasks.h"
#include "utils/data_queue.h"
#include "utils/config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef M_PI
#  define M_PI 3.14159265358979323846f
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Queues */

static sensor_data_packet_t g_sensor_buffer[IMU_SENSOR_QUEUE_SIZE];
static fusion_result_t g_fusion_buffer[IMU_FUSION_QUEUE_SIZE];

data_queue_t g_sensor_queue;
data_queue_t g_fusion_queue;

/* System state */

system_state_t g_system_state =
{
  .running = false,
  .calibration_mode = false,
  .mutex = PTHREAD_MUTEX_INITIALIZER
};

/* Configuration */

static bool g_debug_output = false;
static bool g_verbose_output = false;
static int g_sensor_rate_hz = IMU_SENSOR_RATE_HZ;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: signal_handler
 *
 * Description:
 *   Handle SIGINT and SIGTERM for graceful shutdown
 ****************************************************************************/

static void signal_handler(int signo)
{
  printf("\nReceived signal %d, shutting down...\n", signo);
  g_system_state.running = false;
}

/****************************************************************************
 * Name: print_usage
 ****************************************************************************/

static void print_usage(const char *progname)
{
  printf("\n");
  printf("Usage: %s [OPTIONS]\n", progname);
  printf("\n");
  printf("IMU System Application\n");
  printf("Hardware: 4x ICM42688P (SPI1) + 1x BMM150 (I2C1)\n");
  printf("\n");
  printf("Options:\n");
  printf("  -c          Calibration mode (gyro + mag)\n");
  printf("  -m          Calibrate magnetometer only\n");
  printf("  -t          Run self-test on all sensors\n");
  printf("  -r <rate>   Set sensor rate in Hz (default: %d)\n",
         IMU_SENSOR_RATE_HZ);
  printf("  -d          Enable debug output\n");
  printf("  -v          Enable verbose output\n");
  printf("  -h          Show this help\n");
  printf("\n");
  printf("Examples:\n");
  printf("  %s              # Run with default settings\n", progname);
  printf("  %s -c           # Run calibration first\n", progname);
  printf("  %s -m           # Calibrate mag only\n", progname);
  printf("  %s -r 500 -d    # 500Hz with debug output\n", progname);
  printf("\n");
}

/****************************************************************************
 * Name: init_drivers
 *
 * Description:
 *   Initialize SPI (ICM42688P) and I2C (BMM150) drivers
 ****************************************************************************/

static int init_drivers(void)
{
  int ret;
  int i;

  printf("\n");
  printf("=================================================\n");
  printf("         Initializing Sensor Drivers\n");
  printf("=================================================\n");

  /* Initialize SPI manager for ICM42688P sensors */

  printf("Initializing SPI%d bus...\n", IMU_SPI_BUS);
  ret = spi_manager_init(IMU_SPI_BUS);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize SPI manager: %d\n", ret);
      return ret;
    }
  printf("  ✓ SPI%d initialized\n", IMU_SPI_BUS);

  /* Initialize I2C manager for BMM150 magnetometer */

  printf("Initializing I2C%d bus...\n", IMU_I2C_BUS);
  ret = i2c_manager_init(IMU_I2C_BUS);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize I2C manager: %d\n", ret);
      spi_manager_deinit();
      return ret;
    }
  printf("  ✓ I2C%d initialized (PB6=SCL, PB7=SDA)\n", IMU_I2C_BUS);

  /* Initialize all 4 ICM42688P sensors via SPI */

  printf("\nInitializing ICM42688P sensors (SPI)...\n");
  for (i = 0; i < IMU_NUM_ICM42688P; i++)
    {
      printf("  ICM42688P[%d]...", i);
      fflush(stdout);

      ret = icm42688p_init(i);
      if (ret < 0)
        {
          printf(" FAILED (%d)\n", ret);
          snerr("ERROR: Failed to initialize ICM42688P[%d]: %d\n", i, ret);
          goto errout_icm;
        }

      printf(" OK\n");
    }
  printf("  ✓ All %d ICM42688P sensors initialized\n", IMU_NUM_ICM42688P);

  /* Initialize BMM150 magnetometer via I2C */

  printf("\nInitializing BMM150 magnetometer (I2C)...\n");
  printf("  I2C Address: 0x%02x\n", IMU_BMM150_I2C_ADDR);

  ret = bmm150_init();
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize BMM150 via I2C: %d\n", ret);
      snerr("       Check connections:\n");
      snerr("         - PB6 (I2C1_SCL) -> BMM150 SCL\n");
      snerr("         - PB7 (I2C1_SDA) -> BMM150 SDA\n");
      snerr("         - I2C address: 0x%02x\n", IMU_BMM150_I2C_ADDR);
      goto errout_icm;
    }
  printf("  ✓ BMM150 initialized\n");

  /* Initialize LED driver */

  printf("\nInitializing LED driver...\n");
  ret = led_init();
  if (ret < 0)
    {
      snwarn("WARNING: Failed to initialize LED: %d\n", ret);
      printf("  ⚠ LED initialization failed (non-critical)\n");
      /* Not critical, continue */
    }
  else
    {
      printf("  ✓ LED initialized\n");
    }

  printf("=================================================\n");
  printf("         All Drivers Initialized\n");
  printf("=================================================\n");
  printf("\n");

  return OK;

errout_icm:
  for (i = 0; i < IMU_NUM_ICM42688P; i++)
    {
      icm42688p_deinit(i);
    }

  i2c_manager_deinit();
  spi_manager_deinit();
  return ret;
}

/****************************************************************************
 * Name: deinit_drivers
 ****************************************************************************/

static void deinit_drivers(void)
{
  int i;

  printf("Shutting down drivers...\n");

  led_deinit();
  bmm150_deinit();

  for (i = 0; i < IMU_NUM_ICM42688P; i++)
    {
      icm42688p_deinit(i);
    }

  i2c_manager_deinit();
  spi_manager_deinit();

  printf("All drivers shut down.\n");
}

/****************************************************************************
 * Name: run_self_test
 ****************************************************************************/

static int run_self_test(void)
{
  int ret;
  int i;
  int failures = 0;

  printf("\n");
  printf("=================================================\n");
  printf("           Running Self-Tests\n");
  printf("=================================================\n");

  /* Test ICM42688P sensors */

  for (i = 0; i < IMU_NUM_ICM42688P; i++)
    {
      printf("\nTesting ICM42688P[%d]...\n", i);
      ret = icm42688p_self_test(i);
      if (ret < 0)
        {
          printf("  ✗ FAILED\n");
          failures++;
        }
      else
        {
          printf("  ✓ PASSED\n");
        }
    }

  /* Test BMM150 magnetometer */

  printf("\nTesting BMM150 magnetometer...\n");
  ret = bmm150_self_test();
  if (ret < 0)
    {
      printf("  ✗ FAILED\n");
      failures++;
    }
  else
    {
      printf("  ✓ PASSED\n");
    }

  printf("\n=================================================\n");
  if (failures == 0)
    {
      printf("         All Tests PASSED ✓\n");
    }
  else
    {
      printf("         %d Test(s) FAILED ✗\n", failures);
    }
  printf("=================================================\n");
  printf("\n");

  return (failures == 0) ? OK : -EIO;
}

/****************************************************************************
 * Name: calibrate_sensors
 ****************************************************************************/

static int calibrate_sensors(bool mag_only)
{
  int ret;
  int i;

  printf("\n");
  printf("=================================================\n");
  printf("         Starting Sensor Calibration\n");
  printf("=================================================\n");

  g_system_state.calibration_mode = true;
  led_blink(IMU_LED_SLOW_BLINK_MS);

  /* Calibrate gyros on all 4 IMUs */

  if (!mag_only)
    {
      printf("\nCalibrating Gyroscopes...\n");
      printf("KEEP ALL SENSORS STATIONARY!\n");
      printf("\n");

      for (i = 0; i < IMU_NUM_ICM42688P; i++)
        {
          printf("Calibrating ICM42688P[%d]...\n", i);
          ret = icm42688p_calibrate_gyro(i, IMU_GYRO_CALIB_SAMPLES);
          if (ret < 0)
            {
              snerr("ERROR: Failed to calibrate ICM42688P[%d]: %d\n", i, ret);
              g_system_state.calibration_mode = false;
              return ret;
            }
          printf("  ✓ ICM42688P[%d] calibrated\n", i);
        }
    }

  /* Calibrate magnetometer */

  printf("\nCalibrating Magnetometer...\n");
  ret = bmm150_calibrate();
  if (ret < 0)
    {
      snerr("ERROR: Failed to calibrate BMM150: %d\n", ret);
      g_system_state.calibration_mode = false;
      return ret;
    }

  g_system_state.calibration_mode = false;
  led_set(true);

  printf("\n");
  printf("=================================================\n");
  printf("      Calibration Complete! ✓\n");
  printf("=================================================\n");
  printf("\n");

  sleep(2);

  return OK;
}

/****************************************************************************
 * Name: create_tasks
 ****************************************************************************/

static int create_tasks(void)
{
  pthread_attr_t attr;
  struct sched_param param;
  int ret;

  printf("Creating RTOS tasks...\n");

  /* Initialize pthread attributes */

  pthread_attr_init(&attr);

  /* Create sensor task (highest priority) */

  pthread_attr_setstacksize(&attr, IMU_SENSOR_TASK_STACKSIZE);
  param.sched_priority = IMU_SENSOR_TASK_PRIORITY;
  pthread_attr_setschedparam(&attr, &param);

  ret = pthread_create(&g_system_state.sensor_thread, &attr,
                       sensor_task_main, NULL);
  if (ret != 0)
    {
      snerr("ERROR: Failed to create sensor task: %d\n", ret);
      pthread_attr_destroy(&attr);
      return -ret;
    }
  printf("  ✓ Sensor task created (priority %d)\n",
         IMU_SENSOR_TASK_PRIORITY);

  /* Create fusion task */

  pthread_attr_setstacksize(&attr, IMU_FUSION_TASK_STACKSIZE);
  param.sched_priority = IMU_FUSION_TASK_PRIORITY;
  pthread_attr_setschedparam(&attr, &param);

  ret = pthread_create(&g_system_state.fusion_thread, &attr,
                       fusion_task_main, NULL);
  if (ret != 0)
    {
      snerr("ERROR: Failed to create fusion task: %d\n", ret);
      g_system_state.running = false;
      pthread_join(g_system_state.sensor_thread, NULL);
      pthread_attr_destroy(&attr);
      return -ret;
    }
  printf("  ✓ Fusion task created (priority %d)\n",
         IMU_FUSION_TASK_PRIORITY);

  /* Create LED task */

  pthread_attr_setstacksize(&attr, IMU_LED_TASK_STACKSIZE);
  param.sched_priority = IMU_LED_TASK_PRIORITY;
  pthread_attr_setschedparam(&attr, &param);

  ret = pthread_create(&g_system_state.led_thread, &attr,
                       led_task_main, NULL);
  if (ret != 0)
    {
      snerr("ERROR: Failed to create LED task: %d\n", ret);
      g_system_state.running = false;
      pthread_join(g_system_state.sensor_thread, NULL);
      pthread_join(g_system_state.fusion_thread, NULL);
      pthread_attr_destroy(&attr);
      return -ret;
    }
  printf("  ✓ LED task created (priority %d)\n",
         IMU_LED_TASK_PRIORITY);

  pthread_attr_destroy(&attr);

  printf("All tasks created successfully.\n");
  return OK;
}

/****************************************************************************
 * Name: print_status
 ****************************************************************************/

static void print_status(void)
{
  fusion_result_t result;
  float heading;
  size_t sensor_count;
  size_t fusion_count;
  uint32_t bmm_reads = 0;
  uint32_t bmm_errors = 0;

  /* Get latest fusion result */

  if (data_queue_pop(&g_fusion_queue, &result) == OK)
    {
      heading = orientation_get_heading();

      printf("Orientation - ");
      printf("Roll: %6.2f° ", result.euler.roll * 180.0f / M_PI);
      printf("Pitch: %6.2f° ", result.euler.pitch * 180.0f / M_PI);
      printf("Yaw: %6.2f° ", result.euler.yaw * 180.0f / M_PI);
      printf("Heading: %6.2f°", heading);

      if (g_debug_output)
        {
          printf("\nQuaternion - ");
          printf("W: %6.3f ", result.quaternion.w);
          printf("X: %6.3f ", result.quaternion.x);
          printf("Y: %6.3f ", result.quaternion.y);
          printf("Z: %6.3f", result.quaternion.z);
        }

      printf("\n");
    }

  /* Print queue status and statistics */

  if (g_debug_output || g_verbose_output)
    {
      sensor_count = data_queue_count(&g_sensor_queue);
      fusion_count = data_queue_count(&g_fusion_queue);

      printf("Queues - Sensor: %3zu/%d Fusion: %3zu/%d",
             sensor_count, IMU_SENSOR_QUEUE_SIZE,
             fusion_count, IMU_FUSION_QUEUE_SIZE);

      bmm150_get_stats(&bmm_reads, &bmm_errors);
      printf(" | BMM150: reads=%u errors=%u", bmm_reads, bmm_errors);

      printf("\n");
    }
}

/****************************************************************************
 * Name: print_system_info
 ****************************************************************************/

static void print_system_info(void)
{
  printf("\n");
  printf("╔═══════════════════════════════════════════════╗\n");
  printf("║         IMU System - NuttX RTOS               ║\n");
  printf("╚═══════════════════════════════════════════════╝\n");
  printf("\n");
  printf("Hardware Configuration:\n");
  printf("  MCU: STM32H743 @ %d MHz\n", STM32_SYSCLK_FREQUENCY / 1000000);
  printf("  Sensors: %d x ICM42688P + 1 x BMM150\n", IMU_NUM_ICM42688P);
  printf("\n");
  printf("Communication Interfaces:\n");
  printf("  SPI%d: ICM42688P #0-3 (PA5, PA6, PA7)\n", IMU_SPI_BUS);
  printf("    CS Pins: PA4, PD11, PD12, PD13\n");
  printf("  I2C%d: BMM150 (PB6=SCL, PB7=SDA)\n", IMU_I2C_BUS);
  printf("    Address: 0x%02x\n", IMU_BMM150_I2C_ADDR);
  printf("\n");
  printf("Timing Configuration:\n");
  printf("  Sensor Rate: %d Hz\n", IMU_SENSOR_RATE_HZ);
  printf("  Fusion Rate: %d Hz\n", IMU_FUSION_RATE_HZ);
  printf("  LED Rate: %d Hz\n", IMU_LED_RATE_HZ);
  printf("\n");
  printf("Memory Configuration:\n");
  printf("  Sensor Queue: %d samples × %zu bytes = %zu KB\n",
         IMU_SENSOR_QUEUE_SIZE, sizeof(sensor_data_packet_t),
         (IMU_SENSOR_QUEUE_SIZE * sizeof(sensor_data_packet_t)) / 1024);
  printf("  Fusion Queue: %d samples × %zu bytes = %zu KB\n",
         IMU_FUSION_QUEUE_SIZE, sizeof(fusion_result_t),
         (IMU_FUSION_QUEUE_SIZE * sizeof(fusion_result_t)) / 1024);
  printf("\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  bool do_calibration = false;
  bool do_self_test = false;
  bool mag_only = false;
  int opt;
  int ret;

  /* Parse command line arguments */

  while ((opt = getopt(argc, argv, "cmtr:dvh")) != ERROR)
    {
      switch (opt)
        {
          case 'c':
            do_calibration = true;
            break;

          case 'm':
            mag_only = true;
            do_calibration = true;
            break;

          case 't':
            do_self_test = true;
            break;

          case 'r':
            g_sensor_rate_hz = atoi(optarg);
            if (g_sensor_rate_hz <= 0 || g_sensor_rate_hz > 1000)
              {
                fprintf(stderr, "ERROR: Invalid sensor rate: %s\n", optarg);
                fprintf(stderr, "       Valid range: 1-1000 Hz\n");
                return EXIT_FAILURE;
              }
            break;

          case 'd':
            g_debug_output = true;
            break;

          case 'v':
            g_verbose_output = true;
            break;

          case 'h':
            print_usage(argv[0]);
            return EXIT_SUCCESS;

          default:
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

  /* Print system information */

  print_system_info();

  /* Initialize data queues */

  printf("Initializing data queues...\n");

  ret = data_queue_init(&g_sensor_queue, g_sensor_buffer,
                        sizeof(sensor_data_packet_t),
                        IMU_SENSOR_QUEUE_SIZE);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize sensor queue: %d\n", ret);
      return EXIT_FAILURE;
    }

  ret = data_queue_init(&g_fusion_queue, g_fusion_buffer,
                        sizeof(fusion_result_t),
                        IMU_FUSION_QUEUE_SIZE);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize fusion queue: %d\n", ret);
      return EXIT_FAILURE;
    }

  printf("  ✓ Queues initialized\n");
  printf("\n");

  /* Initialize drivers */

  ret = init_drivers();
  if (ret < 0)
    {
      return EXIT_FAILURE;
    }

  /* Run self-test if requested */

  if (do_self_test)
    {
      ret = run_self_test();
      if (ret < 0)
        {
          printf("\nSelf-test failed. Continue anyway? (y/n): ");
          int c = getchar();
          if (c != 'y' && c != 'Y')
            {
              deinit_drivers();
              return EXIT_FAILURE;
            }
        }
    }

  /* Calibrate sensors if requested */

  if (do_calibration)
    {
      ret = calibrate_sensors(mag_only);
      if (ret < 0)
        {
          deinit_drivers();
          return EXIT_FAILURE;
        }
    }

  /* Setup signal handler for graceful shutdown */

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  /* Create and start tasks */

  g_system_state.running = true;

  ret = create_tasks();
  if (ret < 0)
    {
      deinit_drivers();
      return EXIT_FAILURE;
    }

  printf("\n");
  printf("╔═══════════════════════════════════════════════╗\n");
  printf("║     IMU System Running - Press Ctrl+C to Stop ║\n");
  printf("╚═══════════════════════════════════════════════╝\n");
  printf("\n");

  /* Main loop - print status periodically */

  while (g_system_state.running)
    {
      sleep(1);
      print_status();
    }

  /* Wait for tasks to complete */

  printf("\n");
  printf("Stopping IMU System...\n");

  pthread_join(g_system_state.sensor_thread, NULL);
  pthread_join(g_system_state.fusion_thread, NULL);
  pthread_join(g_system_state.led_thread, NULL);

  printf("All tasks stopped.\n");

  /* Cleanup */

  deinit_drivers();

  printf("\n");
  printf("╔═══════════════════════════════════════════════╗\n");
  printf("║          IMU System Stopped                   ║\n");
  printf("╚═══════════════════════════════════════════════╝\n");
  printf("\n");

  return EXIT_SUCCESS;
}