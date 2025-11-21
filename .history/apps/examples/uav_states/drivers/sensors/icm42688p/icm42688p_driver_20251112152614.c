/****************************************************************************
 * apps/examples/imu_system/drivers/icm42688p/icm42688p_driver.c
 *
 * ICM-42688-P Multi-instance Driver Implementation
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
#include <debug.h>
#include <math.h>
#include <nuttx/clock.h>

#include "icm42688p_driver.h"
#include "icm42688p_regs.h"
#include "../spi/spi_manager.h"
#include "../../utils/config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_DEVICES 4

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  bool initialized;
  uint8_t current_bank;
  icm42688p_scaled_data_t gyro_offset;
  float gyro_sensitivity;
  float accel_sensitivity;
} icm42688p_dev_state_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static icm42688p_dev_state_t g_devices[MAX_DEVICES];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int select_bank(uint8_t devid, uint8_t bank)
{
  int ret;

  if (g_devices[devid].current_bank == bank)
    {
      return OK;
    }

  ret = spi_manager_write_reg(devid, ICM42688P_REG_BANK_SEL, bank);
  if (ret == OK)
    {
      g_devices[devid].current_bank = bank;
    }

  return ret;
}

static int reset_device(uint8_t devid)
{
  int ret;

  ret = select_bank(devid, BANK_0);
  if (ret < 0)
    {
      return ret;
    }

  ret = spi_manager_write_reg(devid, ICM42688P_DEVICE_CONFIG, 0x01);
  if (ret < 0)
    {
      snerr("ERROR: Soft reset failed for device %d\n", devid);
      return ret;
    }

  usleep(100000);  /* 100ms */
  return OK;
}

static int configure_device(uint8_t devid)
{
  int ret;

  ret = select_bank(devid, BANK_0);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable Gyro and Accel in Low Noise mode */

  ret = spi_manager_write_reg(devid, ICM42688P_PWR_MGMT0,
                               PWR_MGMT0_GYRO_MODE_LN |
                               PWR_MGMT0_ACCEL_MODE_LN);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure power for device %d\n", devid);
      return ret;
    }

  usleep(50000);  /* 50ms for sensors to start */

  /* Configure Gyro: ±2000 dps, 1kHz ODR */

  g_devices[devid].gyro_sensitivity = GYRO_SENSITIVITY_2000DPS;

  ret = spi_manager_write_reg(devid, ICM42688P_GYRO_CONFIG0,
                               GYRO_CONFIG0_FS_SEL_2000DPS |
                               GYRO_CONFIG0_ODR_1KHZ);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure gyro for device %d\n", devid);
      return ret;
    }

  /* Configure Accel: ±16g, 1kHz ODR */

  g_devices[devid].accel_sensitivity = ACCEL_SENSITIVITY_16G;

  ret = spi_manager_write_reg(devid, ICM42688P_ACCEL_CONFIG0,
                               ACCEL_CONFIG0_FS_SEL_16G |
                               ACCEL_CONFIG0_ODR_1KHZ);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure accel for device %d\n", devid);
      return ret;
    }

  /* Configure filter bandwidth */

  ret = spi_manager_write_reg(devid, ICM42688P_GYRO_ACCEL_CONFIG0, 0x44);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure filters for device %d\n", devid);
      return ret;
    }

  usleep(50000);  /* 50ms stabilization */

  sninfo("ICM-42688-P[%d] configured\n", devid);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icm42688p_init
 ****************************************************************************/

int icm42688p_init(uint8_t devid)
{
  uint8_t who_am_i;
  int ret;

  if (devid >= MAX_DEVICES)
    {
      return -EINVAL;
    }

  if (g_devices[devid].initialized)
    {
      return -EBUSY;
    }

  memset(&g_devices[devid], 0, sizeof(icm42688p_dev_state_t));

  /* Wait for power-on */

  usleep(100000);  /* 100ms */

  /* Verify WHO_AM_I */

  ret = select_bank(devid, BANK_0);
  if (ret < 0)
    {
      return ret;
    }

  ret = spi_manager_read_reg(devid, ICM42688P_WHO_AM_I, &who_am_i);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read WHO_AM_I for device %d\n", devid);
      return ret;
    }

  if (who_am_i != ICM42688P_WHO_AM_I_VALUE)
    {
      snerr("ERROR: Invalid WHO_AM_I for device %d: 0x%02x "
            "(expected 0x47)\n", devid, who_am_i);
      return -ENODEV;
    }

  sninfo("ICM-42688-P[%d] detected (WHO_AM_I=0x%02x)\n", devid, who_am_i);

  /* Reset device */

  ret = reset_device(devid);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure device */

  ret = configure_device(devid);
  if (ret < 0)
    {
      return ret;
    }

  g_devices[devid].initialized = true;

  sninfo("ICM-42688-P[%d] initialized successfully\n", devid);
  return OK;
}

/****************************************************************************
 * Name: icm42688p_deinit
 ****************************************************************************/

void icm42688p_deinit(uint8_t devid)
{
  if (devid < MAX_DEVICES)
    {
      g_devices[devid].initialized = false;
    }
}

/****************************************************************************
 * Name: icm42688p_read
 ****************************************************************************/

int icm42688p_read(uint8_t devid, icm42688p_data_t *data)
{
  uint8_t buffer[14];
  int16_t raw_temp;
  int16_t raw_accel_x;
  int16_t raw_accel_y;
  int16_t raw_accel_z;
  int16_t raw_gyro_x;
  int16_t raw_gyro_y;
  int16_t raw_gyro_z;
  int ret;

  if (devid >= MAX_DEVICES || !data)
    {
      return -EINVAL;
    }

  if (!g_devices[devid].initialized)
    {
      return -ENODEV;
    }

  /* Select Bank 0 */

  ret = select_bank(devid, BANK_0);
  if (ret < 0)
    {
      return ret;
    }

  /* Burst read all sensor data */

  ret = spi_manager_read_regs(devid, ICM42688P_TEMP_DATA1, buffer, 14);
  if (ret < 0)
    {
      return ret;
    }

  /* Parse data (MSB first) */

  raw_temp = (int16_t)((buffer[0] << 8) | buffer[1]);
  raw_accel_x = (int16_t)((buffer[2] << 8) | buffer[3]);
  raw_accel_y = (int16_t)((buffer[4] << 8) | buffer[5]);
  raw_accel_z = (int16_t)((buffer[6] << 8) | buffer[7]);
  raw_gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
  raw_gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
  raw_gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);

  /* Convert to scaled values */

  data->accel.x = (float)raw_accel_x / g_devices[devid].accel_sensitivity;
  data->accel.y = (float)raw_accel_y / g_devices[devid].accel_sensitivity;
  data->accel.z = (float)raw_accel_z / g_devices[devid].accel_sensitivity;

  data->gyro.x = ((float)raw_gyro_x / g_devices[devid].gyro_sensitivity) -
                 g_devices[devid].gyro_offset.x;
  data->gyro.y = ((float)raw_gyro_y / g_devices[devid].gyro_sensitivity) -
                 g_devices[devid].gyro_offset.y;
  data->gyro.z = ((float)raw_gyro_z / g_devices[devid].gyro_sensitivity) -
                 g_devices[devid].gyro_offset.z;

  data->temperature = ((float)raw_temp / TEMP_SENSITIVITY) + TEMP_OFFSET;
  data->timestamp = clock_systime_ticks() * (1000000 / CLK_TCK);

  return OK;
}

/****************************************************************************
 * Name: icm42688p_calibrate_gyro
 ****************************************************************************/

int icm42688p_calibrate_gyro(uint8_t devid, int num_samples)
{
  icm42688p_data_t data;
  float sum_x = 0;
  float sum_y = 0;
  float sum_z = 0;
  int i;
  int ret;

  if (devid >= MAX_DEVICES || num_samples <= 0)
    {
      return -EINVAL;
    }

  if (!g_devices[devid].initialized)
    {
      return -ENODEV;
    }

  sninfo("Starting gyro calibration for device %d with %d samples...\n",
         devid, num_samples);

  usleep(2000000);  /* 2 seconds to stabilize */

  for (i = 0; i < num_samples; i++)
    {
      ret = icm42688p_read(devid, &data);
      if (ret < 0)
        {
          snerr("ERROR: Calibration read failed at sample %d\n", i);
          return ret;
        }

      sum_x += data.gyro.x + g_devices[devid].gyro_offset.x;
      sum_y += data.gyro.y + g_devices[devid].gyro_offset.y;
      sum_z += data.gyro.z + g_devices[devid].gyro_offset.z;

      usleep(10000);  /* 10ms between samples */
    }

  g_devices[devid].gyro_offset.x = sum_x / num_samples;
  g_devices[devid].gyro_offset.y = sum_y / num_samples;
  g_devices[devid].gyro_offset.z = sum_z / num_samples;

  sninfo("Gyro calibration complete for device %d\n", devid);
  return OK;
}

/****************************************************************************
 * Name: icm42688p_self_test
 ****************************************************************************/

int icm42688p_self_test(uint8_t devid)
{
  icm42688p_data_t data;
  int ret;

  if (devid >= MAX_DEVICES)
    {
      return -EINVAL;
    }

  if (!g_devices[devid].initialized)
    {
      return -ENODEV;
    }

  /* Simple test: read data and verify reasonable values */

  ret = icm42688p_read(devid, &data);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if accel magnitude is roughly 1g (considering orientation) */

  float accel_mag = sqrtf(data.accel.x * data.accel.x +
                          data.accel.y * data.accel.y +
                          data.accel.z * data.accel.z);

  if (accel_mag < 0.5f || accel_mag > 1.5f)
    {
      snerr("ERROR: Self-test failed for device %d: "
            "accel magnitude %.2f\n", devid, accel_mag);
      return -EIO;
    }

  sninfo("Self-test passed for device %d\n", devid);
  return OK;
}
