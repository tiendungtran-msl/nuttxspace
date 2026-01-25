/****************************************************************************
 * apps/examples/uav_states/drivers/sensors/icm42688p/icm42688p_driver.c
 *
 * ICM-42688-P 6-DOF IMU Driver Implementation
 * Sử dụng SPI driver mới
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>
#include <debug.h>
#include <nuttx/clock.h>

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>
#include "stm32_spi_icm.h"

#include "icm42688p_driver.h"
#include "icm42688p_regs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Thời gian chờ sau khi reset (milliseconds) */
#define ICM42688P_RESET_DELAY_MS      100

/* Thời gian chờ sau khi bật nguồn (milliseconds) */
#define ICM42688P_POWERUP_DELAY_MS    50

/* Thời gian chờ sau khi thay đổi bank (microseconds) */
#define ICM42688P_BANK_SWITCH_DELAY_US 1

/****************************************************************************
 * External Functions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Ghi dữ liệu vào thanh ghi ICM42688P */
int icm42688p_write_register(icm42688p_dev_t *dev, uint8_t reg, uint8_t value)
{
  int ret;

  struct spi_sequence_s seq;
  struct spi_trans_s trans;

  uint8_t buffer[2];
  buffer[0] = reg & 0x7F;
  buffer[1] = value;

  if (!dev)
    {
      syslog(LOG_ERR, "ERROR: Invalid device pointer\n");
      return ICM42688P_ERR_INVALID;
    }

  struct spi_dev_s *icm_dev = board_spi1_icm_get_device(dev->id);
  if (icm_dev == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get SPI device for sensor %d\n", dev->id);
      return ICM42688P_ERR_COMM;
    }

  trans.deselect = !true;
  trans.nwords = 2;
  trans.txbuffer = buffer;
  trans.rxbuffer = NULL;

  seq.dev = SPIDEV_ID(SPIDEVTYPE_IMU, dev->id);             
  seq.mode = SPIDEV_MODE3;               
  seq.nbits = 8;             
  seq.ntrans = 1;       
  seq.frequency = 1000000;   
  seq.trans = &trans;

  syslog(LOG_INFO, "Prepare to writing 0x%02X to reg 0x%02X...\n", value, reg);
  ret = spi_transfer(icm_dev, &seq);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: SPI write to reg 0x%02X failed: %d\n", reg, ret);
      return ICM42688P_ERR_COMM;
    }
  syslog(LOG_INFO, "Write 0x%02X to reg 0x%02X successful\n", value, reg);
  return ICM42688P_OK;
}

/* Chọn bank */
static int icm42688p_select_bank(icm42688p_dev_t *dev, uint8_t bank)
{
  int ret;

  if (dev->current_bank == bank)
    {
      return ICM42688P_OK;
    }

  /* Ghi vào thanh ghi BANK_SEL */
  ret = icm42688p_write_register(dev, ICM42688P_REG_BANK_SEL, bank & 0x07);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to select bank %d\n", bank);
      return ICM42688P_ERR_COMM;
    }

  dev->current_bank = bank;
  usleep(ICM42688P_BANK_SWITCH_DELAY_US);

  return ICM42688P_OK;
}

/* Cấu hình ICM42688P */
static int icm42688p_configure(icm42688p_dev_t *dev)
{
  int ret;

  syslog(LOG_INFO, "Configuring ICM42688P...\n");

  /* Chọn Bank 0 */
  ret = icm42688p_select_bank(dev, BANK_0);
  if (ret != ICM42688P_OK)
    {
      return ret;
    }
  syslog(LOG_INFO, "Bank 0 selected\n");
  
  /* Bật Gyro và Accel ở chế độ Low Noise (hiệu suất cao) */
  ret = icm42688p_write_register(dev, ICM42688P_PWR_MGMT0,
                      PWR_MGMT0_GYRO_MODE_LN | PWR_MGMT0_ACCEL_MODE_LN);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure power management\n");
      return ICM42688P_ERR_COMM;
    }
  syslog(LOG_INFO, "Power management configured: Gyro and Accel in Low Noise mode\n");
  usleep(ICM42688P_POWERUP_DELAY_MS * 1000);  /* Chờ sensor khởi động */

  /* Cấu hình Gyroscope: ±2000 dps, ODR 1kHz */
  dev->gyro_fs = GYRO_CONFIG0_FS_SEL_2000DPS;
  dev->gyro_sensitivity = GYRO_SENSITIVITY_2000DPS;

  ret = icm42688p_write_register(dev, ICM42688P_GYRO_CONFIG0,
                      dev->gyro_fs | GYRO_CONFIG0_ODR_1KHZ);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure gyroscope\n");
      return ICM42688P_ERR_COMM;
    }
  syslog(LOG_INFO, "Gyroscope configured: ±2000dps, ODR 1kHz\n");

  /* Cấu hình Accelerometer: ±16g, ODR 1kHz */
  dev->accel_fs = ACCEL_CONFIG0_FS_SEL_16G;
  dev->accel_sensitivity = ACCEL_SENSITIVITY_16G;

  ret = icm42688p_write_register(dev, ICM42688P_ACCEL_CONFIG0,
                      dev->accel_fs | ACCEL_CONFIG0_ODR_1KHZ);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure accelerometer\n");
      return ICM42688P_ERR_COMM;
    }
  syslog(LOG_INFO, "Accelerometer configured: ±16g, ODR 1kHz\n");
  syslog(LOG_INFO, "Filters configured: Low-Pass Filter enabled for Gyro and Accel\n");

  /* Cấu hình bộ lọc (Low-Pass Filter) cho cả Gyro và Accel */
  ret = icm42688p_write_register(dev, ICM42688P_GYRO_ACCEL_CONFIG0, 0x44);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure filters\n");
      return ICM42688P_ERR_COMM;
    }

  usleep(ICM42688P_POWERUP_DELAY_MS * 1000);  /* Chờ ổn định */

  syslog(LOG_INFO, "ICM42688P configured: Gyro ±2000dps, Accel ±16g, ODR 1kHz\n");

  return ICM42688P_OK;
}

/* Đọc dữ liệu từ thanh ghi ICM42688P */
int icm42688p_read_register(icm42688p_dev_t *dev, uint8_t reg, uint8_t *value)
{
  int ret;

  struct spi_sequence_s seq;
  struct spi_trans_s trans;

  uint8_t tx_buffer[2];
  uint8_t rx_buffer[2];
  memset(rx_buffer, 0, sizeof(rx_buffer));

  tx_buffer[0] = reg | 0x80;  /* Đặt bit đọc */
  tx_buffer[1] = 0x00;        /* Byte dummy */
  if (!dev || !value)
    {
      syslog(LOG_ERR, "ERROR: Invalid pointer\n");
      return ICM42688P_ERR_INVALID;
    }
  
  struct spi_dev_s *icm_dev = board_spi1_icm_get_device(dev->id);
  if (icm_dev == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get SPI device for sensor %d\n", dev->id);
      return ICM42688P_ERR_COMM;
    }
  
  trans.deselect = !true;
  trans.nwords = 2;
  trans.txbuffer = tx_buffer;
  trans.rxbuffer = rx_buffer;

  seq.dev = SPIDEV_ID(SPIDEVTYPE_IMU, dev->id);             
  seq.mode = SPIDEV_MODE3;               
  seq.nbits = 8;             
  seq.ntrans = 1;       
  seq.frequency = 1000000;   
  seq.trans = &trans;

  ret = spi_transfer(icm_dev, &seq);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: SPI read from reg 0x%02X failed: %d\n", reg, ret);
      return ICM42688P_ERR_COMM;
    }

  *value = rx_buffer[1];
  return ICM42688P_OK;
}

/* Đọc các dữ liệu thu về từ cảm biến ICM */
int icm42688p_read_registers(icm42688p_dev_t *dev, uint8_t reg, uint8_t *value, size_t length)
{
  int ret;
  uint8_t tx_buffer[length + 1];
  uint8_t rx_buffer[length + 1];
  memset(tx_buffer, 0, sizeof(tx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));

  struct spi_sequence_s seq;
  struct spi_trans_s trans;

  if (!dev)
    {
      syslog(LOG_ERR, "ERROR: Invalid device pointer\n");
      return ICM42688P_ERR_INVALID;
    }

  struct spi_dev_s *icm_dev = board_spi1_icm_get_device(dev->id);
  if (icm_dev == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get SPI device for sensor %d\n", dev->id);
      return ICM42688P_ERR_COMM;
    }
  
  trans.deselect = !true;
  trans.nwords = length + 1;  /* 1 byte reg + 14 bytes data */
  trans.txbuffer = NULL;
  trans.rxbuffer = rx_buffer;

  tx_buffer[0] = reg | 0x80;  /* Đặt bit đọc */
  
  seq.dev = SPIDEV_ID(SPIDEVTYPE_IMU, dev->id);             
  seq.mode = SPIDEV_MODE3;               
  seq.nbits = 8;             
  seq.ntrans = 1;       
  seq.frequency = 1000000;
  seq.trans = &trans;

  ret = spi_transfer(icm_dev, &seq);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: SPI read from reg 0x%02X failed: %d\n", reg, ret);
      return ICM42688P_ERR_COMM;
    }

  memcpy(value, &rx_buffer[1], length);
  return ICM42688P_OK;
}

/* Đặt cảm biến về trạng thái mặc định */
static int icm42688p_reset(icm42688p_dev_t *dev)
{
  int ret;

  if (!dev)
    {
      return ICM42688P_ERR_INVALID;
    }

  syslog(LOG_INFO, "Resetting ICM42688P...\n");

  /* Chọn Bank 0 */
  ret = icm42688p_select_bank(dev, BANK_0);
  if (ret != ICM42688P_OK)
    {
      return ret;
    }
  syslog(LOG_INFO, "Select bank to reset successfully\n");

  /* Gửi lệnh Soft Reset */
  ret = icm42688p_write_register(dev, ICM42688P_DEVICE_CONFIG, 0x01);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Soft reset command failed\n");
      return ICM42688P_ERR_COMM;
    }
  syslog(LOG_INFO, "Soft reset command sent\n");

  /* Chờ reset hoàn thành */
  usleep(ICM42688P_RESET_DELAY_MS * 1000);

  /* Reset current bank */
  dev->current_bank = 0xFF;

  syslog(LOG_INFO, "ICM42688P reset complete\n");

  return ICM42688P_OK;
}

/* Khai báo cảm biến ICM được sử dụng */
int icm42688p_init(icm42688p_dev_t *dev, uint8_t id)
{
  int ret;

  if (!dev)
    {
      syslog(LOG_ERR, "ERROR: Invalid device pointer\n");
      return ICM42688P_ERR_INVALID;
    }

  /* Xóa toàn bộ cấu trúc */
  memset(dev, 0, sizeof(icm42688p_dev_t));

  /* Khởi tạo SPI */
  dev->id = id;
  syslog(LOG_INFO, "Initializing ICM42688P on SPI1, CS%d\n", dev->id);

  /* Chọn Bank 0 */
  dev->current_bank = 0xFF;  /* Force bank switch */
  ret = icm42688p_select_bank(dev, BANK_0);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to select bank 0\n");
      return ret;
    }

  /* Đọc WHO_AM_I để xác nhận thiết bị */
  uint8_t who_am_i;
  ret = icm42688p_read_register(dev, ICM42688P_WHO_AM_I, &who_am_i);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to read WHO_AM_I register\n");
      ret = ICM42688P_ERR_COMM;
      return ret;
    }

  if (who_am_i != ICM42688P_WHO_AM_I_VALUE)
    {
      syslog(LOG_ERR, "ERROR: Invalid WHO_AM_I: 0x%02X (expected 0x%02X)\n",
           who_am_i, ICM42688P_WHO_AM_I_VALUE);
      ret = ICM42688P_ERR_INVALID;
      return ret;
    }

  syslog(LOG_INFO, "ICM42688P detected, WHO_AM_I = 0x%02X\n", who_am_i);

  sleep(1);  /* Chờ ổn định */
  /* Soft reset thiết bị */
  ret = icm42688p_reset(dev);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to reset device\n");
      return ret;
    }
  syslog(LOG_INFO, "ICM42688P reset successfully\n");

  
  /* Cấu hình thiết bị */
  ret = icm42688p_configure(dev);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure device\n");
      return ret;
    }
  syslog(LOG_INFO, "ICM42688P configured successfully\n");

  /* Khởi tạo offset về 0 */
  dev->gyro_offset.x = 0.0f;
  dev->gyro_offset.y = 0.0f;
  dev->gyro_offset.z = 0.0f;

  dev->initialized = true;
  dev->sample_count = 0;

  syslog(LOG_INFO, "ICM42688P initialized successfully\n");

  return ICM42688P_OK;
}

int icm42688p_get_data(icm42688p_dev_t *dev)
{
  int ret;

  if (!dev || !dev->initialized)
    {
      syslog(LOG_ERR, "ERROR: Invalid parameters for get_data\n");
      return ICM42688P_ERR_INVALID;
    }
  
  uint8_t data[14];
  ret = icm42688p_read_registers(dev, ICM42688P_TEMP_DATA1, data, 14);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to read sensor data\n");
      return ret;
    }

  /* Hợp nhất dữ liệu raw từ buffer */
  dev->temp     = (int16_t)((data[0] << 8)  | data[1]);
  dev->accel.x  = (int16_t)((data[2] << 8)  | data[3]);
  dev->accel.y  = (int16_t)((data[4] << 8)  | data[5]);
  dev->accel.z  = (int16_t)((data[6] << 8)  | data[7]);
  dev->gyro.x   = (int16_t)((data[8] << 8)  | data[9]);
  dev->gyro.y   = (int16_t)((data[10] << 8) | data[11]);
  dev->gyro.z   = (int16_t)((data[12] << 8) | data[13]);
  
  /* Chuyển đổi dữ liệu raw sang giá trị thực */
  dev->accel.x = (float)dev->accel.x / dev->accel_sensitivity;
  dev->accel.y = (float)dev->accel.y / dev->accel_sensitivity;
  dev->accel.z = (float)dev->accel.z / dev->accel_sensitivity;

  dev->gyro_calib.x = ((float)dev->gyro.x / dev->gyro_sensitivity) - dev->gyro_offset.x;
  dev->gyro_calib.y = ((float)dev->gyro.y / dev->gyro_sensitivity) - dev->gyro_offset.y;
  dev->gyro_calib.z = ((float)dev->gyro.z / dev->gyro_sensitivity) - dev->gyro_offset.z;
  dev->temp = ((float)dev->temp / TEMP_SENSITIVITY) + TEMP_OFFSET;

  /* Lấy timestamp */
  dev->timestamp = clock_systime_ticks() * (1000000 / CLOCKS_PER_SEC);
  dev->sample_count++;

  return ICM42688P_OK;
}

/* Hiệu chỉnh gyroscope */
int icm42688p_calibrate_gyro(icm42688p_dev_t *dev, int num_samples)
{
  int ret;
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_z = 0.0f;

  if (!dev || !dev->initialized || num_samples <= 0)
    {
      return ICM42688P_ERR_INVALID;
    }

  syslog(LOG_INFO, "Starting gyro calibration with %d samples...\n", num_samples);
  syslog(LOG_INFO, "*** KEEP DEVICE STATIONARY! ***\n");

  /* Chờ 2 giây để ổn định */
  usleep(2000000);

  /* Lấy mẫu và tính trung bình */
  for (int i = 0; i < num_samples; i++)
    {
      ret = icm42688p_get_data(dev);
      if (ret != ICM42688P_OK)
        {
          syslog(LOG_ERR, "ERROR: Calibration read failed at sample %d\n", i);
          return ret;
        }

      /* Cộng dồn (không trừ offset vì đang tính offset) */
      sum_x += dev->gyro.x + dev->gyro_offset.x;
      sum_y += dev->gyro.y + dev->gyro_offset.y;
      sum_z += dev->gyro.z + dev->gyro_offset.z;

      /* Delay giữa các mẫu */
      usleep(10000);  /* 10ms -> ~100Hz */

      /* Hiển thị tiến trình */
      if ((i % 50) == 0)
        {
          printf("Calibration progress: %d/%d\n", i, num_samples);
        }
    }

  /* Tính offset trung bình */
  dev->gyro_offset.x = sum_x / num_samples;
  dev->gyro_offset.y = sum_y / num_samples;
  dev->gyro_offset.z = sum_z / num_samples;

  syslog(LOG_INFO, "Gyro calibration complete!\n");
  syslog(LOG_INFO, "  Offset X: %.3f dps\n", dev->gyro_offset.x);
  syslog(LOG_INFO, "  Offset Y: %.3f dps\n", dev->gyro_offset.y);
  syslog(LOG_INFO, "  Offset Z: %.3f dps\n", dev->gyro_offset.z);

  return ICM42688P_OK;
}

/* Chạy bài kiểm tra tự chẩn đoán */
int icm42688p_self_test(icm42688p_dev_t *dev)
{
  int ret;

  if (!dev || !dev->initialized)
    {
      return ICM42688P_ERR_INVALID;
    }

  syslog(LOG_INFO, "Running ICM42688P self-test...\n");

  /* Đọc một mẫu dữ liệu */
  ret = icm42688p_get_data(dev);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Self-test failed - cannot read data\n");
      return ret;
    }

  /* Kiểm tra giá trị hợp lý */
  /* Accel: Khi đứng yên, tổng vector ~1g */
  float accel_magnitude = sqrtf(dev->accel.x * dev->accel.x +
                                 dev->accel.y * dev->accel.y +
                                 dev->accel.z * dev->accel.z);

  if (accel_magnitude < 0.5f || accel_magnitude > 1.5f)
    {
      syslog(LOG_ERR, "ERROR: Accelerometer magnitude out of range: %.2f g\n",
           accel_magnitude);
      return ICM42688P_ERR_SELFTEST;
    }

  /* Gyro: Khi đứng yên, phải gần 0 */
  float gyro_magnitude = sqrtf(dev->gyro_calib.x * dev->gyro_calib.x +
                                dev->gyro_calib.y * dev->gyro_calib.y +
                                dev->gyro_calib.z * dev->gyro_calib.z);

  if (gyro_magnitude > 50.0f)  /* > 50 dps khi đứng yên là bất thường */
    {
      syslog(LOG_WARNING, "WARNING: Gyro magnitude high: %.2f dps (may need calibration)\n",
            gyro_magnitude);
    }

  /* Kiểm tra nhiệt độ hợp lý */
  if (dev->temp < -40.0f || dev->temp > 85.0f)
    {
      syslog(LOG_ERR, "ERROR: Temperature out of range: %.2f °C\n", dev->temp);
      return ICM42688P_ERR_SELFTEST;
    }

  syslog(LOG_INFO, "Self-test PASSED\n");
  syslog(LOG_INFO, "  Accel magnitude: %.3f g\n", accel_magnitude);
  syslog(LOG_INFO, "  Gyro magnitude:  %.2f dps\n", gyro_magnitude);
  syslog(LOG_INFO, "  Temperature:     %.2f °C\n", dev->temp);

  return ICM42688P_OK;
}

/* In trạng thái hiện tại của cảm biến */
void icm42688p_print_status(icm42688p_dev_t *dev)
{
  if (!dev)
    {
      return;
    }

  printf("\n===== ICM42688P Status =====\n");
  printf("Initialized:   %s\n", dev->initialized ? "YES" : "NO");
  printf("Device ID:     %d\n", dev->id);
  printf("Current Bank:  %d\n", dev->current_bank);
  printf("\n--- Configuration ---\n");
  printf("Gyro Range:    ±2000 dps\n");
  printf("Accel Range:   ±16g\n");
  printf("Gyro Sens:     %.1f LSB/dps\n", dev->gyro_sensitivity);
  printf("Accel Sens:    %.1f LSB/g\n", dev->accel_sensitivity);
  printf("\n--- Calibration ---\n");
  printf("Gyro Offset:   X=%.3f Y=%.3f Z=%.3f dps\n",
         dev->gyro_offset.x, dev->gyro_offset.y, dev->gyro_offset.z);
  printf("\n--- Statistics ---\n");
  printf("Samples Read:  %lu\n", dev->sample_count);
  printf("============================\n\n");
}

/* Tính toán góc pitch, roll theo dữ liệu cảm biến */
int icm42688p_orientation(icm42688p_dev_t *dev)
{
  if (!dev)
    {
      syslog(LOG_ERR, "ERROR: Invalid parameters for orientation calculation\n");
      return ICM42688P_ERR_INVALID;
    }

  /* Tính góc pitch và roll từ dữ liệu accelerometer */
  dev->pitch = atan2f(-dev->accel.x, sqrtf(dev->accel.y * dev->accel.y + dev->accel.z * dev->accel.z)) * (180.0f / M_PI);
  dev->roll  = atan2f(dev->accel.y, dev->accel.z) * (180.0f / M_PI);

  return ICM42688P_OK;
}