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

/* Chọn bank */
static int icm42688p_select_bank(icm42688p_dev_t *dev, uint8_t bank)
{
  int ret;

  if (dev->current_bank == bank)
    {
      return ICM42688P_OK;  /* Đã ở đúng bank */
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

/* Ghi dữ liệu vào thanh ghi ICM42688P */
int icm42688p_write_register(icm42688p_dev_t *dev, uint8_t reg, uint8_t value)
{
  struct spi_sequence_s seq;
  struct spi_trans_s *trans;

  trans->deselect = !true;
  trans->nwords = 2;
  trans-

  seq.dev = board_spi1_icm_get_device(dev->id);             
  seq.mode = SPIDEV_MODE3;               
  seq.nbits = 8;             
  seq.ntrans = 1;       
  seq.frequency = 1000000;   
  seq.trans = &trans; 
  

}

/****************************************************************************
 * Name: icm42688p_convert_accel_data
 *
 * Description:
 *   Chuyển đổi dữ liệu Accelerometer từ raw sang g
 *
 ****************************************************************************/

static void icm42688p_convert_accel_data(icm42688p_dev_t *dev,
                                          int16_t raw_x, int16_t raw_y,
                                          int16_t raw_z,
                                          icm42688p_scaled_data_t *scaled)
{
  scaled->x = (float)raw_x / dev->accel_sensitivity;
  scaled->y = (float)raw_y / dev->accel_sensitivity;
  scaled->z = (float)raw_z / dev->accel_sensitivity;
}

/****************************************************************************
 * Name: icm42688p_convert_gyro_data
 *
 * Description:
 *   Chuyển đổi dữ liệu Gyroscope từ raw sang dps và áp dụng offset
 *
 ****************************************************************************/

static void icm42688p_convert_gyro_data(icm42688p_dev_t *dev,
                                         int16_t raw_x, int16_t raw_y,
                                         int16_t raw_z,
                                         icm42688p_scaled_data_t *scaled)
{
  /* Chuyển đổi và trừ offset hiệu chuẩn */
  scaled->x = ((float)raw_x / dev->gyro_sensitivity) - dev->gyro_offset.x;
  scaled->y = ((float)raw_y / dev->gyro_sensitivity) - dev->gyro_offset.y;
  scaled->z = ((float)raw_z / dev->gyro_sensitivity) - dev->gyro_offset.z;
}

/****************************************************************************
 * Name: icm42688p_convert_temperature
 *
 * Description:
 *   Chuyển đổi dữ liệu nhiệt độ từ raw sang °C
 *
 ****************************************************************************/

static float icm42688p_convert_temperature(int16_t raw_temp)
{
  return ((float)raw_temp / TEMP_SENSITIVITY) + TEMP_OFFSET;
}

/****************************************************************************
 * Name: icm42688p_configure
 *
 * Description:
 *   Cấu hình ICM42688P với các thông số mặc định
 *
 ****************************************************************************/

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
  ret = spi_write_reg(&dev->spi, ICM42688P_PWR_MGMT0,
                      PWR_MGMT0_GYRO_MODE_LN | PWR_MGMT0_ACCEL_MODE_LN);
  if (ret != SPI_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure power management\n");
      return ICM42688P_ERR_COMM;
    }
  syslog(LOG_INFO, "Power management configured: Gyro and Accel in Low Noise mode\n");
  usleep(ICM42688P_POWERUP_DELAY_MS * 1000);  /* Chờ sensor khởi động */

  /* Cấu hình Gyroscope: ±2000 dps, ODR 1kHz */
  dev->gyro_fs = GYRO_CONFIG0_FS_SEL_2000DPS;
  dev->gyro_sensitivity = GYRO_SENSITIVITY_2000DPS;

  ret = spi_write_reg(&dev->spi, ICM42688P_GYRO_CONFIG0,
                      dev->gyro_fs | GYRO_CONFIG0_ODR_1KHZ);
  if (ret != SPI_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure gyroscope\n");
      return ICM42688P_ERR_COMM;
    }
  syslog(LOG_INFO, "Gyroscope configured: ±2000dps, ODR 1kHz\n");

  /* Cấu hình Accelerometer: ±16g, ODR 1kHz */
  dev->accel_fs = ACCEL_CONFIG0_FS_SEL_16G;
  dev->accel_sensitivity = ACCEL_SENSITIVITY_16G;

  ret = spi_write_reg(&dev->spi, ICM42688P_ACCEL_CONFIG0,
                      dev->accel_fs | ACCEL_CONFIG0_ODR_1KHZ);
  if (ret != SPI_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure accelerometer\n");
      return ICM42688P_ERR_COMM;
    }
  syslog(LOG_INFO, "Accelerometer configured: ±16g, ODR 1kHz\n");
  syslog(LOG_INFO, "Filters configured: Low-Pass Filter enabled for Gyro and Accel\n");

  /* Cấu hình bộ lọc (Low-Pass Filter) cho cả Gyro và Accel */
  ret = spi_write_reg(&dev->spi, ICM42688P_GYRO_ACCEL_CONFIG0, 0x44);
  if (ret != SPI_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure filters\n");
      return ICM42688P_ERR_COMM;
    }

  usleep(ICM42688P_POWERUP_DELAY_MS * 1000);  /* Chờ ổn định */

  syslog(LOG_INFO, "ICM42688P configured: Gyro ±2000dps, Accel ±16g, ODR 1kHz\n");

  return ICM42688P_OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icm42688p_init
 ****************************************************************************/

int icm42688p_init(icm42688p_dev_t *dev, uint8_t bus, uint8_t devid)
{
  uint8_t who_am_i;
  int ret;

  if (!dev)
    {
      return ICM42688P_ERR_INVALID;
    }

  /* Xóa toàn bộ cấu trúc */
  memset(dev, 0, sizeof(icm42688p_dev_t));

  /* Khởi tạo SPI */
  syslog(LOG_INFO, "Initializing ICM42688P on SPI%d, CS%d\n",
        bus, devid);

  ret = spi_init(&dev->spi, bus, devid);
  if (ret != SPI_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI: %d\n", ret);
      return ICM42688P_ERR_INIT;
    }

  /* Chờ power-on */
  usleep(100000);  /* 100ms */

  /* Chọn Bank 0 */
  dev->current_bank = 0xFF;  /* Force bank switch */
  ret = icm42688p_select_bank(dev, BANK_0);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to select bank 0\n");
      goto errout;
    }

  /* Đọc WHO_AM_I để xác nhận thiết bị */
  ret = spi_read_reg(&dev->spi, ICM42688P_WHO_AM_I, &who_am_i);
  if (ret != SPI_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to read WHO_AM_I register\n");
      ret = ICM42688P_ERR_COMM;
      goto errout;
    }

  if (who_am_i != ICM42688P_WHO_AM_I_VALUE)
    {
      syslog(LOG_ERR, "ERROR: Invalid WHO_AM_I: 0x%02X (expected 0x%02X)\n",
           who_am_i, ICM42688P_WHO_AM_I_VALUE);
      ret = ICM42688P_ERR_INVALID;
      goto errout;
    }

  syslog(LOG_INFO, "ICM42688P detected, WHO_AM_I = 0x%02X\n", who_am_i);

  /* Soft reset thiết bị */
  ret = icm42688p_reset(dev);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to reset device\n");
      goto errout;
    }

  /* Cấu hình thiết bị */
  ret = icm42688p_configure(dev);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure device\n");
      goto errout;
    }

  /* Khởi tạo offset về 0 */
  dev->gyro_offset.x = 0.0f;
  dev->gyro_offset.y = 0.0f;
  dev->gyro_offset.z = 0.0f;

  dev->initialized = true;
  dev->sample_count = 0;
  dev->error_count = 0;

  syslog(LOG_INFO, "ICM42688P initialized successfully\n");

  return ICM42688P_OK;

errout:
  spi_deinit(&dev->spi);
  return ret;
}

/****************************************************************************
 * Name: icm42688p_deinit
 ****************************************************************************/

int icm42688p_deinit(icm42688p_dev_t *dev)
{
  if (!dev || !dev->initialized)
    {
      return ICM42688P_ERR_INVALID;
    }

  /* Tắt Gyro và Accel */
  icm42688p_select_bank(dev, BANK_0);
  spi_write_reg(&dev->spi, ICM42688P_PWR_MGMT0,
                PWR_MGMT0_GYRO_MODE_OFF | PWR_MGMT0_ACCEL_MODE_OFF);

  /* Đóng SPI */
  spi_deinit(&dev->spi);

  dev->initialized = false;

  syslog(LOG_INFO, "ICM42688P deinitialized\n");

  return ICM42688P_OK;
}

/****************************************************************************
 * Name: icm42688p_reset
 ****************************************************************************/

int icm42688p_reset(icm42688p_dev_t *dev)
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

  /* Gửi lệnh Soft Reset */
  ret = spi_write_reg(&dev->spi, ICM42688P_DEVICE_CONFIG, 0x01);
  if (ret != SPI_OK)
    {
      syslog(LOG_ERR, "ERROR: Soft reset command failed\n");
      return ICM42688P_ERR_COMM;
    }

  /* Chờ reset hoàn thành */
  usleep(ICM42688P_RESET_DELAY_MS * 1000);

  /* Reset current bank */
  dev->current_bank = 0xFF;

  syslog(LOG_INFO, "ICM42688P reset complete\n");

  return ICM42688P_OK;
}

/****************************************************************************
 * Name: icm42688p_read_data
 ****************************************************************************/

int icm42688p_read_data(icm42688p_dev_t *dev, icm42688p_data_t *data)
{
  uint8_t buffer[14];
  int16_t raw_temp;
  int16_t raw_accel_x, raw_accel_y, raw_accel_z;
  int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
  int ret;

  if (!dev || !dev->initialized || !data)
    {
      return ICM42688P_ERR_INVALID;
    }

  /* Chọn Bank 0 */
  ret = icm42688p_select_bank(dev, BANK_0);
  if (ret != ICM42688P_OK)
    {
      return ret;
    }
  syslog(LOG_INFO, "Đã chọn Bank 0\n");

  /* Đọc burst 14 bytes: TEMP + ACCEL_XYZ + GYRO_XYZ */
  ret = spi_read_regs(&dev->spi, ICM42688P_TEMP_DATA1, buffer, 14);
  if (ret != SPI_OK)
    {
      dev->error_count++;
      syslog(LOG_ERR, "ERROR: Failed to read sensor data\n");
      return ICM42688P_ERR_COMM;
    }
  syslog(LOG_INFO, "Đã đọc dữ liệu sensor\n");

  /* Parse dữ liệu (Big-Endian: MSB first) */
  raw_temp     = (int16_t)((buffer[0] << 8) | buffer[1]);
  raw_accel_x  = (int16_t)((buffer[2] << 8) | buffer[3]);
  raw_accel_y  = (int16_t)((buffer[4] << 8) | buffer[5]);
  raw_accel_z  = (int16_t)((buffer[6] << 8) | buffer[7]);
  raw_gyro_x   = (int16_t)((buffer[8] << 8) | buffer[9]);
  raw_gyro_y   = (int16_t)((buffer[10] << 8) | buffer[11]);
  raw_gyro_z   = (int16_t)((buffer[12] << 8) | buffer[13]);

  syslog(LOG_INFO, "Đã đọc dữ liệu sensor\n");

  /* Chuyển đổi sang đơn vị vật lý */
  icm42688p_convert_accel_data(dev, raw_accel_x, raw_accel_y, raw_accel_z,
                                &data->accel);
  icm42688p_convert_gyro_data(dev, raw_gyro_x, raw_gyro_y, raw_gyro_z,
                               &data->gyro);
  data->temperature = icm42688p_convert_temperature(raw_temp);

  /* Lấy timestamp */
  data->timestamp = clock_systime_ticks() * (1000000 / CLOCKS_PER_SEC);

  dev->sample_count++;
  usleep(00);  /* Delay nhỏ để đảm bảo dữ liệu ổn định */
  syslog(LOG_INFO, "Dữ liệu đã được chuyển đổi và timestamp đã được lấy\n");

  return ICM42688P_OK;
}

/****************************************************************************
 * Name: icm42688p_calibrate_gyro
 ****************************************************************************/

int icm42688p_calibrate_gyro(icm42688p_dev_t *dev, int num_samples)
{
  icm42688p_data_t data;
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_z = 0.0f;
  int i;
  int ret;

  if (!dev || !dev->initialized || num_samples <= 0)
    {
      return ICM42688P_ERR_INVALID;
    }

  syslog(LOG_INFO, "Starting gyro calibration with %d samples...\n", num_samples);
  syslog(LOG_INFO, "*** KEEP DEVICE STATIONARY! ***\n");

  /* Chờ 2 giây để ổn định */
  usleep(2000000);

  /* Lấy mẫu và tính trung bình */
  for (i = 0; i < num_samples; i++)
    {
      ret = icm42688p_read_data(dev, &data);
      if (ret != ICM42688P_OK)
        {
          syslog(LOG_ERR, "ERROR: Calibration read failed at sample %d\n", i);
          return ret;
        }

      /* Cộng dồn (không trừ offset vì đang tính offset) */
      sum_x += data.gyro.x + dev->gyro_offset.x;
      sum_y += data.gyro.y + dev->gyro_offset.y;
      sum_z += data.gyro.z + dev->gyro_offset.z;

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

/****************************************************************************
 * Name: icm42688p_self_test
 ****************************************************************************/

int icm42688p_self_test(icm42688p_dev_t *dev)
{
  icm42688p_data_t data;
  int ret;

  if (!dev || !dev->initialized)
    {
      return ICM42688P_ERR_INVALID;
    }

  syslog(LOG_INFO, "Running ICM42688P self-test...\n");

  /* Đọc một mẫu dữ liệu */
  ret = icm42688p_read_data(dev, &data);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "ERROR: Self-test failed - cannot read data\n");
      return ret;
    }

  /* Kiểm tra giá trị hợp lý */
  /* Accel: Khi đứng yên, tổng vector ~1g */
  float accel_magnitude = sqrtf(data.accel.x * data.accel.x +
                                 data.accel.y * data.accel.y +
                                 data.accel.z * data.accel.z);

  if (accel_magnitude < 0.5f || accel_magnitude > 1.5f)
    {
      syslog(LOG_ERR, "ERROR: Accelerometer magnitude out of range: %.2f g\n",
           accel_magnitude);
      return ICM42688P_ERR_SELFTEST;
    }

  /* Gyro: Khi đứng yên, phải gần 0 */
  float gyro_magnitude = sqrtf(data.gyro.x * data.gyro.x +
                                data.gyro.y * data.gyro.y +
                                data.gyro.z * data.gyro.z);

  if (gyro_magnitude > 50.0f)  /* > 50 dps khi đứng yên là bất thường */
    {
      syslog(LOG_WARNING, "WARNING: Gyro magnitude high: %.2f dps (may need calibration)\n",
            gyro_magnitude);
    }

  /* Kiểm tra nhiệt độ hợp lý */
  if (data.temperature < -40.0f || data.temperature > 85.0f)
    {
      syslog(LOG_ERR, "ERROR: Temperature out of range: %.2f °C\n", data.temperature);
      return ICM42688P_ERR_SELFTEST;
    }

  syslog(LOG_INFO, "Self-test PASSED\n");
  syslog(LOG_INFO, "  Accel magnitude: %.3f g\n", accel_magnitude);
  syslog(LOG_INFO, "  Gyro magnitude:  %.2f dps\n", gyro_magnitude);
  syslog(LOG_INFO, "  Temperature:     %.2f °C\n", data.temperature);

  return ICM42688P_OK;
}

/****************************************************************************
 * Name: icm42688p_set_gyro_range
 ****************************************************************************/

int icm42688p_set_gyro_range(icm42688p_dev_t *dev, uint8_t range)
{
  uint8_t fs_sel;
  float sensitivity;
  int ret;

  if (!dev || !dev->initialized)
    {
      return ICM42688P_ERR_INVALID;
    }

  /* Chọn Full Scale và Sensitivity tương ứng */
  switch (range)
    {
      case 0:  /* ±250 dps */
        fs_sel = GYRO_CONFIG0_FS_SEL_250DPS;
        sensitivity = GYRO_SENSITIVITY_250DPS;
        break;

      case 1:  /* ±500 dps */
        fs_sel = GYRO_CONFIG0_FS_SEL_500DPS;
        sensitivity = GYRO_SENSITIVITY_500DPS;
        break;

      case 2:  /* ±1000 dps */
        fs_sel = GYRO_CONFIG0_FS_SEL_1000DPS;
        sensitivity = GYRO_SENSITIVITY_1000DPS;
        break;

      case 3:  /* ±2000 dps */
        fs_sel = GYRO_CONFIG0_FS_SEL_2000DPS;
        sensitivity = GYRO_SENSITIVITY_2000DPS;
        break;

      default:
        return ICM42688P_ERR_INVALID;
    }

  /* Chọn Bank 0 */
  ret = icm42688p_select_bank(dev, BANK_0);
  if (ret != ICM42688P_OK)
    {
      return ret;
    }

  /* Đọc giá trị hiện tại */
  uint8_t reg_val;
  ret = spi_read_reg(&dev->spi, ICM42688P_GYRO_CONFIG0, &reg_val);
  if (ret != SPI_OK)
    {
      return ICM42688P_ERR_COMM;
    }

  /* Giữ ODR, thay đổi FS_SEL */
  reg_val = (reg_val & 0x1F) | fs_sel;

  ret = spi_write_reg(&dev->spi, ICM42688P_GYRO_CONFIG0, reg_val);
  if (ret != SPI_OK)
    {
      return ICM42688P_ERR_COMM;
    }

  /* Cập nhật cấu hình */
  dev->gyro_fs = fs_sel;
  dev->gyro_sensitivity = sensitivity;

  syslog(LOG_INFO, "Gyro range set to %d (sensitivity: %.1f LSB/dps)\n",
        range, sensitivity);

  return ICM42688P_OK;
}

/****************************************************************************
 * Name: icm42688p_set_accel_range
 ****************************************************************************/

int icm42688p_set_accel_range(icm42688p_dev_t *dev, uint8_t range)
{
  uint8_t fs_sel;
  float sensitivity;
  int ret;

  if (!dev || !dev->initialized)
    {
      return ICM42688P_ERR_INVALID;
    }

  /* Chọn Full Scale và Sensitivity */
  switch (range)
    {
      case 0:  /* ±2g */
        fs_sel = ACCEL_CONFIG0_FS_SEL_2G;
        sensitivity = ACCEL_SENSITIVITY_2G;
        break;

      case 1:  /* ±4g */
        fs_sel = ACCEL_CONFIG0_FS_SEL_4G;
        sensitivity = ACCEL_SENSITIVITY_4G;
        break;

      case 2:  /* ±8g */
        fs_sel = ACCEL_CONFIG0_FS_SEL_8G;
        sensitivity = ACCEL_SENSITIVITY_8G;
        break;

      case 3:  /* ±16g */
        fs_sel = ACCEL_CONFIG0_FS_SEL_16G;
        sensitivity = ACCEL_SENSITIVITY_16G;
        break;

      default:
        return ICM42688P_ERR_INVALID;
    }

  /* Chọn Bank 0 */
  ret = icm42688p_select_bank(dev, BANK_0);
  if (ret != ICM42688P_OK)
    {
      return ret;
    }

  /* Đọc giá trị hiện tại */
  uint8_t reg_val;
  ret = spi_read_reg(&dev->spi, ICM42688P_ACCEL_CONFIG0, &reg_val);
  if (ret != SPI_OK)
    {
      return ICM42688P_ERR_COMM;
    }

  /* Giữ ODR, thay đổi FS_SEL */
  reg_val = (reg_val & 0x1F) | fs_sel;

  ret = spi_write_reg(&dev->spi, ICM42688P_ACCEL_CONFIG0, reg_val);
  if (ret != SPI_OK)
    {
      return ICM42688P_ERR_COMM;
    }

  /* Cập nhật cấu hình */
  dev->accel_fs = fs_sel;
  dev->accel_sensitivity = sensitivity;

  syslog(LOG_INFO, "Accel range set to %d (sensitivity: %.1f LSB/g)\n",
        range, sensitivity);

  return ICM42688P_OK;
}

/****************************************************************************
 * Name: icm42688p_set_odr
 ****************************************************************************/

int icm42688p_set_odr(icm42688p_dev_t *dev, uint8_t odr_val)
{
  uint8_t gyro_reg, accel_reg;
  int ret;

  if (!dev || !dev->initialized)
    {
      return ICM42688P_ERR_INVALID;
    }

  /* Chọn Bank 0 */
  ret = icm42688p_select_bank(dev, BANK_0);
  if (ret != ICM42688P_OK)
    {
      return ret;
    }

  /* Đọc GYRO_CONFIG0 */
  ret = spi_read_reg(&dev->spi, ICM42688P_GYRO_CONFIG0, &gyro_reg);
  if (ret != SPI_OK)
    {
      return ICM42688P_ERR_COMM;
    }

  /* Thay đổi ODR, giữ FS_SEL */
  gyro_reg = (gyro_reg & 0xF0) | (odr_val & 0x0F);

  ret = spi_write_reg(&dev->spi, ICM42688P_GYRO_CONFIG0, gyro_reg);
  if (ret != SPI_OK)
    {
      return ICM42688P_ERR_COMM;
    }

  /* Đọc ACCEL_CONFIG0 */
  ret = spi_read_reg(&dev->spi, ICM42688P_ACCEL_CONFIG0, &accel_reg);
  if (ret != SPI_OK)
    {
      return ICM42688P_ERR_COMM;
    }

  /* Thay đổi ODR */
  accel_reg = (accel_reg & 0xF0) | (odr_val & 0x0F);

  ret = spi_write_reg(&dev->spi, ICM42688P_ACCEL_CONFIG0, accel_reg);
  if (ret != SPI_OK)
    {
      return ICM42688P_ERR_COMM;
    }

  syslog(LOG_INFO, "ODR set to 0x%02X\n", odr_val);

  return ICM42688P_OK;
}

/****************************************************************************
 * Name: icm42688p_print_status
 ****************************************************************************/

void icm42688p_print_status(icm42688p_dev_t *dev)
{
  if (!dev)
    {
      return;
    }

  printf("\n===== ICM42688P Status =====\n");
  printf("Initialized:   %s\n", dev->initialized ? "YES" : "NO");
  printf("SPI Bus:       %d\n", dev->spi.bus);
  printf("Device ID:     %d\n", dev->spi.devid);
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
  printf("Errors:        %lu\n", dev->error_count);
  printf("============================\n\n");
}