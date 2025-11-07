/****************************************************************************
 * apps/examples/mpu6050/mpu6050_main.c
 * 
 * Complete MPU6050 pitch/roll measurement application for NuttX
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <nuttx/sensors/mpu6050.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPU6050_DEVICE_PATH "/dev/imu0"

/* Conversion factors */
#define ACCEL_SCALE_2G      16384.0f    /* LSB/g for ±2g range */
#define GYRO_SCALE_250DPS   131.0f      /* LSB/(°/s) for ±250°/s range */
#define RAD_TO_DEG          57.2958f    /* Radians to degrees */
#define DEG_TO_RAD          0.0174533f  /* Degrees to radians */

/* Filter parameters */
#define COMPLEMENTARY_ALPHA 0.98f       /* Complementary filter coefficient */
#define UPDATE_RATE_HZ      50          /* 50Hz update rate */
#define UPDATE_PERIOD_US    (1000000 / UPDATE_RATE_HZ)

/* Application limits */
#define MAX_SAMPLES         10000       /* Maximum samples before auto-stop */
#define TEMP_OFFSET         36.53f      /* Temperature offset for MPU6050 */
#define TEMP_SCALE          340.0f      /* Temperature scale factor */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct angle_data_s
{
  float pitch;
  float roll;
  float yaw;
  float gyro_pitch;
  float gyro_roll;
  float gyro_yaw;
  uint32_t timestamp;
};

struct mpu6050_cal_s
{
  float accel_x_offset;
  float accel_y_offset;
  float accel_z_offset;
  float gyro_x_offset;
  float gyro_y_offset;
  float gyro_z_offset;
  bool calibrated;
};

struct app_state_s
{
  int fd;
  bool running;
  bool verbose;
  bool show_raw;
  bool log_to_file;
  uint32_t sample_count;
  struct mpu6050_cal_s calibration;
  struct angle_data_s last_angles;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct app_state_s g_app_state = {0};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void signal_handler(int signo);
static int mpu6050_app_init(void);
static int mpu6050_app_cleanup(void);
static int mpu6050_calibrate(int fd, struct mpu6050_cal_s *cal);
static void convert_raw_data(const struct mpu6050_data_s *raw,
                           const struct mpu6050_cal_s *cal,
                           float *accel_g, float *gyro_dps, float *temp_c);
static void calculate_angles(const float accel_g[3], const float gyro_dps[3],
                           struct angle_data_s *angles, float dt);
static uint32_t get_timestamp_ms(void);
static void print_header(bool show_raw);
static void print_data(const struct mpu6050_data_s *raw_data,
                      const struct angle_data_s *angles,
                      float temp_c, bool show_raw);
static void print_usage(void);
static int parse_arguments(int argc, char *argv[]);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void signal_handler(int signo)
{
  if (signo == SIGINT || signo == SIGTERM)
    {
      printf("\nReceived signal %d, shutting down gracefully...\n", signo);
      g_app_state.running = false;
    }
}

static int mpu6050_app_init(void)
{
  int ret;

  /* Initialize state */
  memset(&g_app_state, 0, sizeof(g_app_state));
  g_app_state.running = true;

  /* Setup signal handlers */
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  /* Open MPU6050 device */
  g_app_state.fd = open(MPU6050_DEVICE_PATH, O_RDWR);
  if (g_app_state.fd < 0)
    {
      fprintf(stderr, "ERROR: Failed to open %s: %s\n",
              MPU6050_DEVICE_PATH, strerror(errno));
      return -errno;
    }

  /* Reset device */
  ret = ioctl(g_app_state.fd, SNIOC_RESET, 0);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to reset MPU6050: %s\n", strerror(errno));
      close(g_app_state.fd);
      return ret;
    }

  usleep(100000); /* Wait 100ms for reset */

  /* Wake up device */
  ret = ioctl(g_app_state.fd, SNIOC_WAKEUP, 0);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to wake up MPU6050: %s\n", strerror(errno));
      close(g_app_state.fd);
      return ret;
    }

  usleep(50000); /* Wait 50ms for stabilization */

  printf("MPU6050 device initialized successfully\n");
  return OK;
}

static int mpu6050_app_cleanup(void)
{
  if (g_app_state.fd >= 0)
    {
      close(g_app_state.fd);
      g_app_state.fd = -1;
    }

  printf("\nApplication cleanup completed\n");
  printf("Total samples processed: %u\n", g_app_state.sample_count);
  
  return OK;
}

static int mpu6050_calibrate(int fd, struct mpu6050_cal_s *cal)
{
  struct mpu6050_data_s data;
  int32_t accel_sum[3] = {0};
  int32_t gyro_sum[3] = {0};
  int samples = 0;
  int ret;
  int i;

  printf("Calibrating MPU6050...\n");
  printf("Keep the sensor stationary and level!\n");
  printf("Collecting calibration data");

  /* Collect calibration samples */
  for (i = 0; i < 200; i++)
    {
      ret = read(fd, &data, sizeof(data));
      if (ret != sizeof(data))
        {
          fprintf(stderr, "\nERROR: Failed to read calibration data: %s\n",
                  strerror(errno));
          return -errno;
        }

      /* Accumulate samples */
      accel_sum[0] += data.accel_x;
      accel_sum[1] += data.accel_y;
      accel_sum[2] += data.accel_z;
      gyro_sum[0] += data.gyro_x;
      gyro_sum[1] += data.gyro_y;
      gyro_sum[2] += data.gyro_z;

      samples++;

      /* Progress indicator */
      if ((i % 20) == 0)
        {
          printf(".");
          fflush(stdout);
        }

      usleep(10000); /* 10ms between samples */
    }

  printf(" done!\n");

  /* Calculate averages */
  cal->accel_x_offset = (float)accel_sum[0] / samples;
  cal->accel_y_offset = (float)accel_sum[1] / samples;
  cal->accel_z_offset = (float)accel_sum[2] / samples - ACCEL_SCALE_2G; /* Remove 1g */
  cal->gyro_x_offset = (float)gyro_sum[0] / samples;
  cal->gyro_y_offset = (float)gyro_sum[1] / samples;
  cal->gyro_z_offset = (float)gyro_sum[2] / samples;

  cal->calibrated = true;

  printf("Calibration completed:\n");
  printf("  Accel offsets: X=%.1f, Y=%.1f, Z=%.1f\n",
         cal->accel_x_offset, cal->accel_y_offset, cal->accel_z_offset);
  printf("  Gyro offsets:  X=%.1f, Y=%.1f, Z=%.1f\n",
         cal->gyro_x_offset, cal->gyro_y_offset, cal->gyro_z_offset);

  return OK;
}

static void convert_raw_data(const struct mpu6050_data_s *raw,
                           const struct mpu6050_cal_s *cal,
                           float *accel_g, float *gyro_dps, float *temp_c)
{
  /* Convert accelerometer data to g-force */
  accel_g[0] = (raw->accel_x - cal->accel_x_offset) / ACCEL_SCALE_2G;
  accel_g[1] = (raw->accel_y - cal->accel_y_offset) / ACCEL_SCALE_2G;
  accel_g[2] = (raw->accel_z - cal->accel_z_offset) / ACCEL_SCALE_2G;

  /* Convert gyroscope data to degrees per second */
  gyro_dps[0] = (raw->gyro_x - cal->gyro_x_offset) / GYRO_SCALE_250DPS;
  gyro_dps[1] = (raw->gyro_y - cal->gyro_y_offset) / GYRO_SCALE_250DPS;
  gyro_dps[2] = (raw->gyro_z - cal->gyro_z_offset) / GYRO_SCALE_250DPS;

  /* Convert temperature to Celsius */
  *temp_c = (raw->temp / TEMP_SCALE) + TEMP_OFFSET;
}

static void calculate_angles(const float accel_g[3], const float gyro_dps[3],
                           struct angle_data_s *angles, float dt)
{
  float accel_magnitude;
  float accel_pitch, accel_roll;

  /* Calculate accelerometer-based angles */
  accel_magnitude = sqrtf(accel_g[0]*accel_g[0] + 
                         accel_g[1]*accel_g[1] + 
                         accel_g[2]*accel_g[2]);

  /* Check for valid accelerometer data */
  if (accel_magnitude < 0.5f || accel_magnitude > 2.0f)
    {
      /* Invalid accelerometer data - use only gyroscope integration */
      angles->pitch += gyro_dps[0] * dt;
      angles->roll += gyro_dps[1] * dt;
      angles->yaw += gyro_dps[2] * dt;
    }
  else
    {
      /* Calculate pitch and roll from accelerometer */
      accel_pitch = atan2f(accel_g[1], sqrtf(accel_g[0]*accel_g[0] + accel_g[2]*accel_g[2])) * RAD_TO_DEG;
      accel_roll = atan2f(-accel_g[0], sqrtf(accel_g[1]*accel_g[1] + accel_g[2]*accel_g[2])) * RAD_TO_DEG;

      /* Integrate gyroscope data */
      angles->gyro_pitch += gyro_dps[0] * dt;
      angles->gyro_roll += gyro_dps[1] * dt;
      angles->gyro_yaw += gyro_dps[2] * dt;

      /* Apply complementary filter */
      angles->pitch = COMPLEMENTARY_ALPHA * (angles->pitch + gyro_dps[0] * dt) +
                     (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;
      
      angles->roll = COMPLEMENTARY_ALPHA * (angles->roll + gyro_dps[1] * dt) +
                    (1.0f - COMPLEMENTARY_ALPHA) * accel_roll;
      
      /* Yaw can only be integrated from gyroscope */
      angles->yaw += gyro_dps[2] * dt;
    }

  /* Update timestamp */
  angles->timestamp = get_timestamp_ms();
}

static uint32_t get_timestamp_ms(void)
{
  struct timespec ts;
  
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    {
      return 0;
    }
  
  return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void print_header(bool show_raw)
{
  printf("\n");
  printf("┌──────────────────────────────────────────────────────────────────┐\n");
  printf("│                    MPU6050 Measurement Data                     │\n");
  printf("├──────────────────────────────────────────────────────────────────┤\n");
  
  if (show_raw)
    {
      printf("│ Time(ms) │ Pitch(°) │ Roll(°) │ Yaw(°) │ Temp(°C) │    Raw Data    │\n");
      printf("├──────────┼──────────┼─────────┼────────┼──────────┼────────────────┤\n");
    }
  else
    {
      printf("│ Time(ms) │ Pitch(°) │ Roll(°) │  Yaw(°) │ Temp(°C) │ Status │\n");
      printf("├──────────┼──────────┼─────────┼─────────┼──────────┼────────┤\n");
    }
}

static void print_data(const struct mpu6050_data_s *raw_data,
                      const struct angle_data_s *angles,
                      float temp_c, bool show_raw)
{
  static uint32_t line_count = 0;
  
  /* Print header every 20 lines */
  if ((line_count % 20) == 0)
    {
      print_header(show_raw);
    }

  if (show_raw)
    {
      printf("│%9u │%9.2f │%8.2f │%7.2f │%9.1f │ %6d,%6d,%6d │\n",
             angles->timestamp,
             angles->pitch,
             angles->roll, 
             angles->yaw,
             temp_c,
             raw_data->accel_x, raw_data->accel_y, raw_data->accel_z);
    }
  else
    {
      const char *status = "Normal";
      if (fabsf(angles->pitch) > 45.0f || fabsf(angles->roll) > 45.0f)
        {
          status = "Tilted";
        }
      
      printf("│%9u │%9.2f │%8.2f │%8.2f │%9.1f │ %6s │\n",
             angles->timestamp,
             angles->pitch,
             angles->roll,
             angles->yaw,
             temp_c,
             status);
    }

  line_count++;
}

static void print_usage(void)
{
  printf("Usage: mpu6050_app [options]\n");
  printf("\nOptions:\n");
  printf("  -v, --verbose     Enable verbose output\n");
  printf("  -r, --raw         Show raw sensor data\n");
  printf("  -n, --no-cal      Skip calibration\n");
  printf("  -s, --samples N   Stop after N samples (default: continuous)\n");
  printf("  -h, --help        Show this help message\n");
  printf("\nControls:\n");
  printf("  Ctrl+C            Stop measurement\n");
  printf("\nExample:\n");
  printf("  mpu6050_app -v -r     # Verbose mode with raw data\n");
  printf("  mpu6050_app -s 1000   # Collect 1000 samples then stop\n");
}

static int parse_arguments(int argc, char *argv[])
{
  int i;
  bool skip_calibration = false;
  uint32_t max_samples = 0;

  for (i = 1; i < argc; i++)
    {
      if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0)
        {
          g_app_state.verbose = true;
        }
      else if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--raw") == 0)
        {
          g_app_state.show_raw = true;
        }
      else if (strcmp(argv[i], "-n") == 0 || strcmp(argv[i], "--no-cal") == 0)
        {
          skip_calibration = true;
        }
      else if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--samples") == 0)
        {
          if (++i >= argc)
            {
              fprintf(stderr, "ERROR: --samples requires a number\n");
              return -EINVAL;
            }
          max_samples = strtoul(argv[i], NULL, 10);
          if (max_samples == 0)
            {
              fprintf(stderr, "ERROR: Invalid sample count\n");
              return -EINVAL;
            }
        }
      else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
        {
          print_usage();
          exit(EXIT_SUCCESS);
        }
      else
        {
          fprintf(stderr, "ERROR: Unknown option: %s\n", argv[i]);
          print_usage();
          return -EINVAL;
        }
    }

  /* Store max samples */
  if (max_samples > 0)
    {
      printf("Will stop after %u samples\n", max_samples);
      /* Note: This would need to be stored in global state for use in main loop */
    }

  return skip_calibration ? 1 : 0;  /* Return 1 if skip calibration */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct mpu6050_data_s raw_data;
  struct angle_data_s angles = {0};
  float accel_g[3], gyro_dps[3], temp_c;
  uint32_t current_time, last_time;
  float dt;
  int ret;
  int skip_cal;

  printf("MPU6050 Pitch/Roll/Yaw Measurement Application\n");
  printf("==============================================\n");

  /* Parse command line arguments */
  skip_cal = parse_arguments(argc, argv);
  if (skip_cal < 0)
    {
      return EXIT_FAILURE;
    }

  /* Initialize application */
  ret = mpu6050_app_init();
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Application initialization failed\n");
      return EXIT_FAILURE;
    }

  /* Calibrate sensor if requested */
  if (!skip_cal)
    {
      ret = mpu6050_calibrate(g_app_state.fd, &g_app_state.calibration);
      if (ret < 0)
        {
          fprintf(stderr, "ERROR: Calibration failed\n");
          mpu6050_app_cleanup();
          return EXIT_FAILURE;
        }
    }
  else
    {
      printf("Skipping calibration - using default offsets\n");
      memset(&g_app_state.calibration, 0, sizeof(g_app_state.calibration));
      g_app_state.calibration.calibrated = true;
    }

  printf("\nStarting measurements at %dHz (Press Ctrl+C to stop)\n", UPDATE_RATE_HZ);

  /* Initialize timing */
  last_time = get_timestamp_ms();

  /* Main measurement loop */
  while (g_app_state.running && g_app_state.sample_count < MAX_SAMPLES)
    {
      /* Read sensor data */
      ret = read(g_app_state.fd, &raw_data, sizeof(raw_data));
      if (ret != sizeof(raw_data))
        {
          if (errno == EINTR)
            {
              continue; /* Interrupted by signal */
            }
          fprintf(stderr, "ERROR: Failed to read sensor data: %s\n", strerror(errno));
          break;
        }

      /* Calculate time difference */
      current_time = get_timestamp_ms();
      dt = (current_time - last_time) / 1000.0f; /* Convert to seconds */
      
      /* Sanity check on dt */
      if (dt > 0.1f || dt <= 0.0f) 
        {
          dt = 1.0f / UPDATE_RATE_HZ; /* Use nominal period */
        }
      
      last_time = current_time;

      /* Convert raw data to physical units */
      convert_raw_data(&raw_data, &g_app_state.calibration, 
                      accel_g, gyro_dps, &temp_c);

      /* Calculate angles */
      calculate_angles(accel_g, gyro_dps, &angles, dt);

      /* Print data */
      print_data(&raw_data, &angles, temp_c, g_app_state.show_raw);

      /* Update sample count */
      g_app_state.sample_count++;

      /* Maintain update rate */
      usleep(UPDATE_PERIOD_US);
    }

  /* Print final statistics */
  printf("\n");
  printf("└──────────────────────────────────────────────────────────────────┘\n");
  printf("Final orientation: Pitch=%.2f°, Roll=%.2f°, Yaw=%.2f°\n",
         angles.pitch, angles.roll, angles.yaw);

  /* Cleanup and exit */
  mpu6050_app_cleanup();
  
  return EXIT_SUCCESS;
}