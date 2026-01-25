/****************************************************************************
 * apps/examples/icm42688p_test/icm42688p_test.c
 *
 * Test tối ưu cho ICM-42688-P với rate control
 * Thu thập dữ liệu 1000Hz, nhưng chỉ hiển thị theo tần suất hợp lý
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include <debug.h>
#include <errno.h>
#include <syslog.h>
#include <signal.h>
#include <stdbool.h>
#include <nuttx/clock.h>

#include "../drivers/sensors/icm42688p/icm42688p_driver.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ICM42688P_DEV0_ID              0
#define ICM42688P_DEV1_ID              1

/* Tần số thu thập dữ liệu (Hz) */
#define SAMPLE_RATE_HZ                 1000

/* Tần số hiển thị (Hz) - QUAN TRỌNG: Không nên > 50Hz */
#define DISPLAY_RATE_HZ                10

/* Số samples giữa các lần hiển thị */
#define DISPLAY_DECIMATION             (SAMPLE_RATE_HZ / DISPLAY_RATE_HZ)

/* Microseconds giữa mỗi sample */
#define SAMPLE_PERIOD_US               (1000000 / SAMPLE_RATE_HZ)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile bool g_running = true;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Handle Ctrl+C để dừng chương trình gracefully */
static void signal_handler(int signo)
{
  if (signo == SIGINT)
    {
      g_running = false;
    }
}

/* In thống kê sau khi kết thúc */
static void print_statistics(icm42688p_dev_t *dev, uint32_t total_samples, uint64_t start_time_us, uint64_t end_time_us)
{
  float elapsed_sec = (end_time_us - start_time_us) / 1000000.0f;
  float actual_rate = total_samples / elapsed_sec;
  
  printf("\n\n========== Test Statistics ==========\n");
  printf("Total samples:     %lu\n", total_samples);
  printf("Elapsed time:      %.2f seconds\n", elapsed_sec);
  printf("Target rate:       %d Hz\n", SAMPLE_RATE_HZ);
  printf("Actual rate:       %.2f Hz\n", actual_rate);
  printf("Error count:       %lu\n", dev->error_count);
  printf("Success rate:      %.2f%%\n", 
         100.0f * (total_samples - dev->error_count) / total_samples);
  printf("=====================================\n\n");
}

/****************************************************************************
 * Name: test_continuous_sampling
 * 
 * Description:
 *   Thu thập dữ liệu liên tục với rate control
 ****************************************************************************/

static int test_continuous_sampling(icm42688p_dev_t *dev)
{
  int ret;
  uint32_t sample_count = 0;
  uint32_t display_count = 0;
  uint32_t error_consecutive = 0;
  uint64_t start_time_us;
  uint64_t end_time_us;
  uint64_t next_sample_time_us;
  uint64_t current_time_us;
  
  /* Thiết lập signal handler */
  signal(SIGINT, signal_handler);
  
  printf("\n=== Continuous Data Acquisition ===\n");
  printf("Sample rate:   %d Hz\n", SAMPLE_RATE_HZ);
  printf("Display rate:  %d Hz\n", DISPLAY_RATE_HZ);
  printf("Press Ctrl+C to stop\n\n");
  
  /* Header cho bảng dữ liệu */
  printf("     Accel (g)            Gyro (dps)          Temp   Sample#  Rate\n");
  printf("   X      Y      Z       X      Y      Z      (°C)            (Hz)\n");
  printf("-----------------------------------------------------------------------\n");
  fflush(stdout);
  
  /* Delay nhỏ để console sẵn sàng */
  usleep(100000);
  
  /* Lấy thời điểm bắt đầu */
  start_time_us = clock_systime_ticks() * (1000000 / CLOCKS_PER_SEC);
  next_sample_time_us = start_time_us;
  
  /* Main sampling loop */
  while (g_running)
    {
      /* Đọc dữ liệu sensor */
      ret = icm42688p_get_data(dev);
      
      if (ret != ICM42688P_OK)
        {
          error_consecutive++;
          
          /* Nếu lỗi liên tục quá 10 lần, dừng */
          if (error_consecutive > 10)
            {
              syslog(LOG_ERR, "Too many consecutive errors, stopping\n");
              break;
            }
          
          /* Retry với delay nhỏ */
          usleep(1000);
          continue;
        }
      
      /* Reset consecutive error counter khi đọc thành công */
      error_consecutive = 0;
      sample_count++;
      
      /* Hiển thị dữ liệu theo tần số DISPLAY_RATE_HZ */
      if (sample_count % DISPLAY_DECIMATION == 0)
        {
          current_time_us = clock_systime_ticks() * (1000000 / CLOCKS_PER_SEC);
          float elapsed_sec = (current_time_us - start_time_us) / 1000000.0f;
          float actual_rate = (elapsed_sec > 0) ? sample_count / elapsed_sec : 0;
          
          /* In dòng dữ liệu - dùng \r để overwrite nếu muốn */
          printf("%+6.3f %+6.3f %+6.3f  %+7.2f %+7.2f %+7.2f  %6.2f  %7lu  %6.1f\n",
                 dev->accel.x, dev->accel.y, dev->accel.z,
                 dev->gyro_calib.x, dev->gyro_calib.y, dev->gyro_calib.z,
                 dev->temp,
                 sample_count,
                 actual_rate);
          
          fflush(stdout);
          display_count++;
          
          /* Mỗi 10 dòng, in lại header để dễ đọc */
          if (display_count % 20 == 0)
            {
              printf("-----------------------------------------------------------------------\n");
              fflush(stdout);
            }
        }
      
      /* Timing control - đảm bảo đúng sample rate */
      next_sample_time_us += SAMPLE_PERIOD_US;
      current_time_us = clock_systime_ticks() * (1000000 / CLOCKS_PER_SEC);
      
      if (next_sample_time_us > current_time_us)
        {
          int64_t sleep_us = next_sample_time_us - current_time_us;
          
          /* Chỉ sleep nếu thời gian chờ hợp lý (< 2ms) */
          if (sleep_us > 0 && sleep_us < 2000)
            {
              usleep(sleep_us);
            }
        }
      else
        {
          /* Nếu chậm hơn, reset timing để tránh drift */
          next_sample_time_us = current_time_us;
        }
    }
  
  /* Lấy thời điểm kết thúc */
  end_time_us = clock_systime_ticks() * (1000000 / CLOCKS_PER_SEC);
  
  /* In thống kê */
  print_statistics(dev, sample_count, start_time_us, end_time_us);
  
  return ICM42688P_OK;
}

/****************************************************************************
 * Name: test_single_read
 * 
 * Description:
 *   Test đọc đơn để verify hoạt động
 ****************************************************************************/

static int test_single_read(icm42688p_dev_t *dev)
{
  int ret;
  
  printf("\n=== Single Read Test ===\n");
  
  for (int i = 0; i < 5; i++)
    {
      ret = icm42688p_get_data(dev);
      if (ret != ICM42688P_OK)
        {
          printf("Read %d: FAILED (error=%d)\n", i + 1, ret);
          return ret;
        }
      
      printf("Read %d: Accel=(%.3f, %.3f, %.3f) Gyro=(%.2f, %.2f, %.2f) Temp=%.2f°C\n",
             i + 1,
             dev->accel.x, dev->accel.y, dev->accel.z,
             dev->gyro_calib.x, dev->gyro_calib.y, dev->gyro_calib.z,
             dev->temp);
      
      usleep(100000);  /* 100ms between reads */
    }
  
  printf("Single read test: PASSED\n");
  return ICM42688P_OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icm42688p_test_main
 ****************************************************************************/

int icm42688p_test_main(int argc, char *argv[])
{
  icm42688p_dev_t dev0;
  int ret;
  
  /* Clear screen effect */
  printf("\n\n");
  fflush(stdout);
  
  printf("====================================================\n");
  printf("  ICM-42688-P IMU Test Application\n");
  printf("====================================================\n\n");
  
  /* Stabilization delay */
  printf("Initializing...\n");
  usleep(500000);
  
  /* Initialize sensor */
  ret = icm42688p_init(&dev0, ICM42688P_DEV0_ID);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: ICM42688P initialization failed (error=%d)\n", ret);
      return -1;
    }
  
  printf("Initialization: SUCCESS\n");
  
  /* Print device status */
  icm42688p_print_status(&dev0);
  
  /* Run single read test */
  ret = test_single_read(&dev0);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Single read test failed\n");
      goto cleanup;
    }
  
  /* Optional: Run self-test */
  printf("\n=== Running Self-Test ===\n");
  ret = icm42688p_self_test(&dev0);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_WARNING, "WARNING: Self-test failed, but continuing...\n");
    }
  else
    {
      printf("Self-test: PASSED\n");
    }
  
  /* Optional: Calibrate gyro */
  printf("\n=== Gyro Calibration ===\n");
  printf("Keep device stationary for 5 seconds...\n");
  usleep(2000000);
  
  ret = icm42688p_calibrate_gyro(&dev0, 500);  /* 500 samples @ 100Hz = 5 sec */
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_WARNING, "WARNING: Calibration failed, using zero offset\n");
    }
  
  /* Main continuous sampling */
  ret = test_continuous_sampling(&dev0);
  
cleanup:
  /* Cleanup */
  printf("\nCleaning up...\n");
  icm42688p_deinit(&dev0);
  
  printf("\nTest completed.\n\n");
  
  return (ret == ICM42688P_OK) ? 0 : -1;
}