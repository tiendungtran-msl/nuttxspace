/****************************************************************************
 * apps/examples/imu_system/drivers/bmm150/bmm150_driver.c
 *
 * BMM150 Magnetometer Driver Implementation
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

#include "bmm150_driver.h"
#include "bmm150_regs.h"
#include "../spi/spi_manager.h"
#include "../../utils/config.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  int8_t dig_x1;
  int8_t dig_y1;
  int8_t dig_x2;
  int8_t dig_y2;
  uint16_t dig_z1;
  int16_t dig_z2;
  int16_t dig_z3;
  int16_t dig_z4;
  uint8_t dig_xy1;
  int8_t dig_xy2;
  uint16_t dig_xyz1;
} bmm150_trim_data_t;

typedef struct
{
  bool initialized;
  bmm150_trim_data_t trim;
  float offset_x;
  float offset_y;
  float offset_z;
  float scale_x;
  float scale_y;
  float scale_z;
} bmm150_dev_state_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bmm150_dev_state_t g_bmm150;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int read_trim_registers(void)
{
  uint8_t data[2];
  int ret;

  ret = spi_manager_read_reg(IMU_BMM150_DEVID, BMM150_REG_DIG_X1,
                              (uint8_t *)&g_bmm150.trim.dig_x1);
  if (ret < 0) return ret;

  ret = spi_manager_read_reg(IMU_BMM150_DEVID, BMM150_REG_DIG_Y1,
                              (uint8_t *)&g_bmm150.trim.dig_y1);
  if (ret < 0) return ret;

  ret = spi_manager_read_reg(IMU_BMM150_DEVID, BMM150_REG_DIG_X2,
                              (uint8_t *)&g_bmm150.trim.dig_x2);
  if (ret < 0) return ret;

  ret = spi_manager_read_reg(IMU_BMM150_DEVID, BMM150_REG_DIG_Y2,
                              (uint8_t *)&g_bmm150.trim.dig_y2);
  if (ret < 0) return ret;

  ret = spi_manager_read_reg(IMU_BMM150_DEVID, BMM150_REG_DIG_XY1,
                              &g_bmm150.trim.dig_xy1);
  if (ret < 0) return ret;

  ret = spi_manager_read_reg(IMU_BMM150_DEVID, BMM150_REG_DIG_XY2,
                              (uint8_t *)&g_bmm150.trim.dig_xy2);
  if (ret < 0) return ret;

  ret = spi_manager_read_regs(IMU_BMM150_DEVID, BMM150_REG_DIG_Z1_LSB,
                               data, 2);
  if (ret < 0) return ret;
  g_bmm150.trim.dig_z1 = (uint16_t)(data[0] | (data[1] << 8));

  ret = spi_manager_read_regs(IMU_BMM150_DEVID, BMM150_REG_DIG_Z2_LSB,
                               data, 2);
  if (ret < 0) return ret;
  g_bmm150.trim.dig_z2 = (int16_t)(data[0] | (data[1] << 8));

  ret = spi_manager_read_regs(IMU_BMM150_DEVID, BMM150_REG_DIG_Z3_LSB,
                               data, 2);
  if (ret < 0) return ret;
  g_bmm150.trim.dig_z3 = (int16_t)(data[0] | (data[1] << 8));

  ret = spi_manager_read_regs(IMU_BMM150_DEVID, BMM150_REG_DIG_Z4_LSB,
                               data, 2);
  if (ret < 0) return ret;
  g_bmm150.trim.dig_z4 = (int16_t)(data[0] | (data[1] << 8));

  ret = spi_manager_read_regs(IMU_BMM150_DEVID, BMM150_REG_DIG_XYZ1_LSB,
                               data, 2);
  if (ret < 0) return ret;
  g_bmm150.trim.dig_xyz1 = (uint16_t)(data[0] | (data[1] << 8));

  return OK;
}

static float compensate_x(int16_t mag_data_x, uint16_t data_rhall)
{
  float retval;
  float process_comp_x0;
  float process_comp_x1;
  float process_comp_x2;
  float process_comp_x3;
  float process_comp_x4;

  if (mag_data_x != -4096)
    {
      if (data_rhall != 0)
        {
          process_comp_x0 = ((float)g_bmm150.trim.dig_xyz1) * 16384.0f /
                            data_rhall;
          retval = process_comp_x0 - 16384.0f;
          process_comp_x1 = ((float)g_bmm150.trim.dig_xy2) *
                            (retval * retval / 268435456.0f);
          process_comp_x2 = process_comp_x1 + retval *
                            ((float)g_bmm150.trim.dig_xy1) / 16384.0f;
          process_comp_x3 = ((float)g_bmm150.trim.dig_x2) + 160.0f;
          process_comp_x4 = mag_data_x * ((process_comp_x2 + 256.0f) *
                            process_comp_x3);
          retval = ((process_comp_x4 / 8192.0f) +
                    (((float)g_bmm150.trim.dig_x1) * 8.0f)) / 16.0f;
        }
      else
        {
          retval = 0.0f;
        }
    }
  else
    {
      retval = 0.0f;
    }

  return retval;
}

static float compensate_y(int16_t mag_data_y, uint16_t data_rhall)
{
  float retval;
  float process_comp_y0;
  float process_comp_y1;
  float process_comp_y2;
  float process_comp_y3;
  float process_comp_y4;

  if (mag_data_y != -4096)
    {
      if (data_rhall != 0)
        {
          process_comp_y0 = ((float)g_bmm150.trim.dig_xyz1) * 16384.0f /
                            data_rhall;
          retval = process_comp_y0 - 16384.0f;
          process_comp_y1 = ((float)g_bmm150.trim.dig_xy2) *
                            (retval * retval / 268435456.0f);
          process_comp_y2 = process_comp_y1 + retval *
                            ((float)g_bmm150.trim.dig_xy1) / 16384.0f;
          process_comp_y3 = ((float)g_bmm150.trim.dig_y2) + 160.0f;
          process_comp_y4 = mag_data_y * (((process_comp_y2) + 256.0f) *
                            process_comp_y3);
          retval = ((process_comp_y4 / 8192.0f) +
                    (((float)g_bmm150.trim.dig_y1) * 8.0f)) / 16.0f;
        }
      else
        {
          retval = 0.0f;
        }
    }
  else
    {
      retval = 0.0f;
    }

  return retval;
}

static float compensate_z(int16_t mag_data_z, uint16_t data_rhall)
{
  float retval;
  float process_comp_z0;
  float process_comp_z1;
  float process_comp_z2;
  float process_comp_z3;
  float process_comp_z4;

  if (mag_data_z != -16384)
    {
      if ((data_rhall != 0) && (g_bmm150.trim.dig_z2 != 0) &&
          (g_bmm150.trim.dig_z1 != 0) && (g_bmm150.trim.dig_xyz1 != 0))
        {
          process_comp_z0 = ((float)mag_data_z) -
                            ((float)g_bmm150.trim.dig_z4);
          process_comp_z1 = ((float)data_rhall) -
                            ((float)g_bmm150.trim.dig_xyz1);
          process_comp_z2 = (((float)g_bmm150.trim.dig_z3) *
                            process_comp_z1);
          process_comp_z3 = ((float)g_bmm150.trim.dig_z1) *
                            ((float)data_rhall) / 32768.0f;
          process_comp_z4 = ((float)g_bmm150.trim.dig_z2) +
                            process_comp_z3;
          retval = (process_comp_z0 * 131072.0f - process_comp_z2) /
                   ((process_comp_z4) * 4.0f) / 16.0f;
        }
      else
        {
          retval = 0.0f;
        }
    }
  else
    {
      retval = 0.0f;
    }

  return retval;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmm150_init
 ****************************************************************************/

int bmm150_init(void)
{
  uint8_t chip_id;
  int ret;

  if (g_bmm150.initialized)
    {
      return -EBUSY;
    }

  memset(&g_bmm150, 0, sizeof(bmm150_dev_state_t));

  /* Default calibration values */

  g_bmm150.scale_x = 1.0f;
  g_bmm150.scale_y = 1.0f;
  g_bmm150.scale_z = 1.0f;

  /* Read chip ID */

  ret = spi_manager_read_reg(IMU_BMM150_DEVID, BMM150_REG_CHIP_ID,
                              &chip_id);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read BMM150 chip ID\n");
      return ret;
    }

  if (chip_id != BMM150_CHIP_ID)
    {
      snerr("ERROR: Invalid BMM150 chip ID: 0x%02x (expected 0x32)\n",
            chip_id);
      return -ENODEV;
    }

  sninfo("BMM150 detected (CHIP_ID=0x%02x)\n", chip_id);

  /* Power on */

  ret = spi_manager_write_reg(IMU_BMM150_DEVID, BMM150_REG_PWR_CTRL,
                               BMM150_PWR_CTRL_POWER_ON);
  if (ret < 0)
    {
      snerr("ERROR: Failed to power on BMM150\n");
      return ret;
    }

  usleep(5000);  /* 5ms startup time */

  /* Read trim registers */

  ret = read_trim_registers();
  if (ret < 0)
    {
      snerr("ERROR: Failed to read trim registers\n");
      return ret;
    }

  /* Set to normal mode with 25 Hz ODR */

  ret = spi_manager_write_reg(IMU_BMM150_DEVID, BMM150_REG_OP_MODE,
                               BMM150_OP_MODE_NORMAL |
                               BMM150_DATA_RATE_25HZ);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set operation mode\n");
      return ret;
    }

  /* Set repetition for X/Y and Z */

  ret = spi_manager_write_reg(IMU_BMM150_DEVID, BMM150_REG_REP_XY,
                               BMM150_REP_XY_REGULAR);
  if (ret < 0)
    {
      return ret;
    }

  ret = spi_manager_write_reg(IMU_BMM150_DEVID, BMM150_REG_REP_Z,
                               BMM150_REP_Z_REGULAR);
  if (ret < 0)
    {
      return ret;
    }

  usleep(10000);  /* 10ms stabilization */

  g_bmm150.initialized = true;

  sninfo("BMM150 initialized successfully\n");
  return OK;
}

/****************************************************************************
 * Name: bmm150_deinit
 ****************************************************************************/

void bmm150_deinit(void)
{
  if (g_bmm150.initialized)
    {
      spi_manager_write_reg(IMU_BMM150_DEVID, BMM150_REG_PWR_CTRL,
                            BMM150_PWR_CTRL_POWER_OFF);
      g_bmm150.initialized = false;
    }
}

/****************************************************************************
 * Name: bmm150_read
 ****************************************************************************/

int bmm150_read(bmm150_data_t *data)
{
  uint8_t buffer[8];
  int16_t raw_x;
  int16_t raw_y;
  int16_t raw_z;
  uint16_t raw_rhall;
  int ret;

  if (!data)
    {
      return -EINVAL;
    }

  if (!g_bmm150.initialized)
    {
      return -ENODEV;
    }

  /* Read all data registers */

  ret = spi_manager_read_regs(IMU_BMM150_DEVID, BMM150_REG_DATA_X_LSB,
                               buffer, 8);
  if (ret < 0)
    {
      return ret;
    }

  /* Extract 13-bit values */

  raw_x = (int16_t)(((int16_t)((int8_t)buffer[1])) * 32) |
          ((buffer[0] & 0xF8) >> 3);
  raw_y = (int16_t)(((int16_t)((int8_t)buffer[3])) * 32) |
          ((buffer[2] & 0xF8) >> 3);
  raw_z = (int16_t)(((int16_t)((int8_t)buffer[5])) * 128) |
          ((buffer[4] & 0xFE) >> 1);
  raw_rhall = (uint16_t)(((uint16_t)buffer[7] << 6) |
                         ((buffer[6] & 0xFC) >> 2));

  /* Compensate using trim values */

  data->x = compensate_x(raw_x, raw_rhall);
  data->y = compensate_y(raw_y, raw_rhall);
  data->z = compensate_z(raw_z, raw_rhall);

  /* Apply calibration */

  data->x = (data->x - g_bmm150.offset_x) * g_bmm150.scale_x;
  data->y = (data->y - g_bmm150.offset_y) * g_bmm150.scale_y;
  data->z = (data->z - g_bmm150.offset_z) * g_bmm150.scale_z;

  data->timestamp = clock_systime_ticks() * (1000000 / CLK_TCK);

  return OK;
}

/****************************************************************************
 * Name: bmm150_calibrate
 ****************************************************************************/

int bmm150_calibrate(void)
{
  bmm150_data_t data;
  float min_x = 10000.0f;
  float max_x = -10000.0f;
  float min_y = 10000.0f;
  float max_y = -10000.0f;
  float min_z = 10000.0f;
  float max_z = -10000.0f;
  int i;
  int ret;

  if (!g_bmm150.initialized)
    {
      return -ENODEV;
    }

  sninfo("Starting magnetometer calibration...\n");
  sninfo("Rotate the sensor in all directions for %d samples\n",
         IMU_MAG_CALIB_SAMPLES);

  /* Collect samples while rotating */

  for (i = 0; i < IMU_MAG_CALIB_SAMPLES; i++)
    {
      ret = bmm150_read(&data);
      if (ret < 0)
        {
          snerr("ERROR: Calibration read failed\n");
          return ret;
        }

      /* Track min/max for each axis */

      if (data.x < min_x) min_x = data.x;
      if (data.x > max_x) max_x = data.x;
      if (data.y < min_y) min_y = data.y;
      if (data.y > max_y) max_y = data.y;
      if (data.z < min_z) min_z = data.z;
      if (data.z > max_z) max_z = data.z;

      usleep(40000);  /* 40ms between samples (25 Hz) */
    }

  /* Calculate hard iron offsets and soft iron scale factors */

  g_bmm150.offset_x = (max_x + min_x) / 2.0f;
  g_bmm150.offset_y = (max_y + min_y) / 2.0f;
  g_bmm150.offset_z = (max_z + min_z) / 2.0f;

  float avg_delta = ((max_x - min_x) + (max_y - min_y) +
                     (max_z - min_z)) / 3.0f;

  g_bmm150.scale_x = avg_delta / (max_x - min_x);
  g_bmm150.scale_y = avg_delta / (max_y - min_y);
  g_bmm150.scale_z = avg_delta / (max_z - min_z);

  sninfo("Magnetometer calibration complete:\n");
  sninfo("  Offset: (%.2f, %.2f, %.2f)\n",
         g_bmm150.offset_x, g_bmm150.offset_y, g_bmm150.offset_z);
  sninfo("  Scale: (%.3f, %.3f, %.3f)\n",
         g_bmm150.scale_x, g_bmm150.scale_y, g_bmm150.scale_z);

  return OK;
}
