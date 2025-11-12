/****************************************************************************
 * apps/examples/imu_system/imu_system_main.c
 *
 * IMU System Main Entry Point
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

#include "drivers/spi/spi_manager.h"
#include "drivers/icm42688p/icm42688p_driver.h"
#include "drivers/bmm150/bmm150_driver.h"
#include "drivers/led/led_driver.h"
#include "algorithms/sensor_fusion.h"
#include "algorithms/orientation.h"
#include "tasks/tasks.h"
#include "utils/data_queue.h"
#include "utils/config.h"

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
static int g_sensor_rate_hz = IMU_SENSOR_RATE_HZ;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void signal_handler(int signo)
{
  g_system_state.running = false;
}

static void print_usage(const char *progname)
{
  printf("Usage: %s [OPTIONS]\n", progname);
  printf("Options:\n");
  printf("  -c          Calibration mode\n");
  printf("  -r <rate>   Set sensor rate in Hz (default: %d)\n",
         IMU_SENSOR_RATE_HZ);
  printf("  -d          Enable debug output\n");
  printf("  -h          Show this help\n");
}

static int init_drivers(void)
{
  int ret;
  int i;

  /* Initialize SPI manager */

  ret = spi_manager_init(IMU_SPI_BUS);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize SPI manager: %d\n", ret);
      return ret;
    }

  /* Initialize all 4 ICM42688P sensors */

  for (i = 0; i < IMU_NUM_ICM42688P; i++)
    {
      ret = icm42688p_init(i);
      if (ret < 0)
        {
          snerr("ERROR: Failed to initialize ICM42688P[%d]: %d\n", i, ret);
          goto errout_icm;
        }
    }

  /* Initialize BMM150 magnetometer */

  ret = bmm150_init();
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize BMM150: %d\n", ret);
      goto errout_icm;
    }

  /* Initialize LED driver */

  ret = led_init();
  if (ret < 0)
    {
      snerr("WARNING: Failed to initialize LED: %d\n", ret);
      /* Not critical, continue */
    }

  return OK;

errout_icm:
  for (i = 0; i < IMU_NUM_ICM42688P; i++)
    {
      icm42688p_deinit(i);
    }

  spi_manager_deinit();
  return ret;
}

static void deinit_drivers(void)
{
  int i;

  led_deinit();
  bmm150_deinit();

  for (i = 0; i < IMU_NUM_ICM42688P; i++)
    {
      icm42688p_deinit(i);
    }

  spi_manager_deinit();
}

static int calibrate_sensors(void)
{
  int ret;
  int i;

  printf("Starting sensor calibration...\n");
  printf("Keep all sensors stationary!\n");

  g_system_state.calibration_mode = true;
  led_blink(IMU_LED_SLOW_BLINK_MS);

  /* Calibrate gyros on all 4 IMUs */

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
    }

  /* Calibrate magnetometer */

  printf("Calibrating BMM150...\n");
  printf("Rotate the sensor in all directions!\n");
  ret = bmm150_calibrate();
  if (ret < 0)
    {
      snerr("ERROR: Failed to calibrate BMM150: %d\n", ret);
      g_system_state.calibration_mode = false;
      return ret;
    }

  g_system_state.calibration_mode = false;
  led_set(true);
  printf("Calibration complete!\n");

  return OK;
}

static int create_tasks(void)
{
  pthread_attr_t attr;
  struct sched_param param;
  int ret;

  /* Initialize pthread attributes */

  pthread_attr_init(&attr);

  /* Create sensor task */

  pthread_attr_setstacksize(&attr, IMU_SENSOR_TASK_STACKSIZE);
  param.sched_priority = IMU_SENSOR_TASK_PRIORITY;
  pthread_attr_setschedparam(&attr, &param);

  ret = pthread_create(&g_system_state.sensor_thread, &attr,
                       sensor_task_main, NULL);
  if (ret != 0)
    {
      snerr("ERROR: Failed to create sensor task: %d\n", ret);
      return -ret;
    }

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
      return -ret;
    }

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
      return -ret;
    }

  pthread_attr_destroy(&attr);

  return OK;
}

static void print_status(void)
{
  fusion_result_t result;
  float heading;
  size_t sensor_count;
  size_t fusion_count;

  /* Get latest fusion result */

  if (data_queue_pop(&g_fusion_queue, &result) == OK)
    {
      heading = orientation_get_heading();

      printf("Orientation - ");
      printf("Roll: %6.2f° ", result.euler.roll * 180.0f / M_PI);
      printf("Pitch: %6.2f° ", result.euler.pitch * 180.0f / M_PI);
      printf("Yaw: %6.2f° ", result.euler.yaw * 180.0f / M_PI);
      printf("Heading: %6.2f°\n", heading);

      if (g_debug_output)
        {
          printf("Quaternion - ");
          printf("W: %6.3f ", result.quaternion.w);
          printf("X: %6.3f ", result.quaternion.x);
          printf("Y: %6.3f ", result.quaternion.y);
          printf("Z: %6.3f\n", result.quaternion.z);
        }
    }

  /* Print queue status */

  sensor_count = data_queue_count(&g_sensor_queue);
  fusion_count = data_queue_count(&g_fusion_queue);

  if (g_debug_output)
    {
      printf("Queues - Sensor: %zu/%d Fusion: %zu/%d\n",
             sensor_count, IMU_SENSOR_QUEUE_SIZE,
             fusion_count, IMU_FUSION_QUEUE_SIZE);
    }
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
  int opt;
  int ret;

  /* Parse command line arguments */

  while ((opt = getopt(argc, argv, "cr:dh")) != ERROR)
    {
      switch (opt)
        {
          case 'c':
            do_calibration = true;
            break;

          case 'r':
            g_sensor_rate_hz = atoi(optarg);
            if (g_sensor_rate_hz <= 0 || g_sensor_rate_hz > 1000)
              {
                fprintf(stderr, "ERROR: Invalid sensor rate: %s\n", optarg);
                return EXIT_FAILURE;
              }

            break;

          case 'd':
            g_debug_output = true;
            break;

          case 'h':
            print_usage(argv[0]);
            return EXIT_SUCCESS;

          default:
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

  printf("IMU System Starting...\n");
  printf("Configuration:\n");
  printf("  Sensors: %d x ICM42688P + 1 x BMM150\n", IMU_NUM_ICM42688P);
  printf("  Sensor Rate: %d Hz\n", IMU_SENSOR_RATE_HZ);
  printf("  Fusion Rate: %d Hz\n", IMU_FUSION_RATE_HZ);

  /* Initialize data queues */

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

  /* Initialize drivers */

  ret = init_drivers();
  if (ret < 0)
    {
      return EXIT_FAILURE;
    }

  /* Calibrate sensors if requested */

  if (do_calibration)
    {
      ret = calibrate_sensors();
      if (ret < 0)
        {
          deinit_drivers();
          return EXIT_FAILURE;
        }
    }

  /* Setup signal handler */

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

  printf("IMU System running. Press Ctrl+C to stop.\n");

  /* Main loop - print status periodically */

  while (g_system_state.running)
    {
      sleep(1);
      print_status();
    }

  /* Wait for tasks to complete */

  printf("\nStopping IMU System...\n");

  pthread_join(g_system_state.sensor_thread, NULL);
  pthread_join(g_system_state.fusion_thread, NULL);
  pthread_join(g_system_state.led_thread, NULL);

  /* Cleanup */

  deinit_drivers();

  printf("IMU System stopped.\n");

  return EXIT_SUCCESS;
}
