/****************************************************************************
 * apps/examples/uav_states/drivers/sensors/bmm150/bmm150_driver.c
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
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/clock.h>
#include <float.h>


#include "bmm150_driver.h"
#include "bmm150_regs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#define BMM150_STARTUP_TIME_MS      3    /* Power-on startup time */
#define BMM150_SOFT_RESET_DELAY_MS  1    /* Soft reset delay */
#define BMM150_FORCED_MODE_DELAY_MS 4    /* Forced mode measurement delay */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmm150_soft_reset
 ****************************************************************************/

static int bmm150_soft_reset(bmm150_dev_t *dev)
{
  int ret;

  ret = i2c_write_reg(&dev->i2c, BMM150_REG_PWR_CTRL, 
                      BMM150_PWR_CTRL_SOFT_RESET);
  if (ret != I2C_OK)
    {
      return BMM150_ERR_COMM;
    }

  usleep(BMM150_SOFT_RESET_DELAY_MS * 1000);
  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_read_trim_registers
 ****************************************************************************/

static int bmm150_read_trim_registers(bmm150_dev_t *dev)
{
  uint8_t data[2];
  int ret;

  /* Read all trim registers */

  ret = i2c_read_reg(&dev->i2c, BMM150_REG_DIG_X1, 
                     (uint8_t *)&dev->trim.dig_x1);
  if (ret != I2C_OK) return BMM150_ERR_COMM;

  ret = i2c_read_reg(&dev->i2c, BMM150_REG_DIG_Y1, 
                     (uint8_t *)&dev->trim.dig_y1);
  if (ret != I2C_OK) return BMM150_ERR_COMM;

  ret = i2c_read_reg(&dev->i2c, BMM150_REG_DIG_X2, 
                     (uint8_t *)&dev->trim.dig_x2);
  if (ret != I2C_OK) return BMM150_ERR_COMM;

  ret = i2c_read_reg(&dev->i2c, BMM150_REG_DIG_Y2, 
                     (uint8_t *)&dev->trim.dig_y2);
  if (ret != I2C_OK) return BMM150_ERR_COMM;

  ret = i2c_read_reg(&dev->i2c, BMM150_REG_DIG_XY1, 
                     &dev->trim.dig_xy1);
  if (ret != I2C_OK) return BMM150_ERR_COMM;

  ret = i2c_read_reg(&dev->i2c, BMM150_REG_DIG_XY2, 
                     (uint8_t *)&dev->trim.dig_xy2);
  if (ret != I2C_OK) return BMM150_ERR_COMM;

  /* Read 16-bit values */

  ret = i2c_read_regs(&dev->i2c, BMM150_REG_DIG_Z1_LSB, data, 2);
  if (ret != I2C_OK) return BMM150_ERR_COMM;
  dev->trim.dig_z1 = (uint16_t)(data[0] | (data[1] << 8));

  ret = i2c_read_regs(&dev->i2c, BMM150_REG_DIG_Z2_LSB, data, 2);
  if (ret != I2C_OK) return BMM150_ERR_COMM;
  dev->trim.dig_z2 = (int16_t)(data[0] | (data[1] << 8));

  ret = i2c_read_regs(&dev->i2c, BMM150_REG_DIG_Z3_LSB, data, 2);
  if (ret != I2C_OK) return BMM150_ERR_COMM;
  dev->trim.dig_z3 = (int16_t)(data[0] | (data[1] << 8));

  ret = i2c_read_regs(&dev->i2c, BMM150_REG_DIG_Z4_LSB, data, 2);
  if (ret != I2C_OK) return BMM150_ERR_COMM;
  dev->trim.dig_z4 = (int16_t)(data[0] | (data[1] << 8));

  ret = i2c_read_regs(&dev->i2c, BMM150_REG_DIG_XYZ1_LSB, data, 2);
  if (ret != I2C_OK) return BMM150_ERR_COMM;
  dev->trim.dig_xyz1 = (uint16_t)(data[0] | (data[1] << 8));

  _info("BMM150 trim data loaded successfully\n");
  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_compensate_x
 ****************************************************************************/

static float bmm150_compensate_x(bmm150_dev_t *dev, int16_t raw_x, 
                                  uint16_t raw_rhall)
{
  float retval;
  float process_comp_x0;
  float process_comp_x1;
  float process_comp_x2;
  float process_comp_x3;
  float process_comp_x4;

  /* Processing compensation equations */

  process_comp_x0 = (((float)dev->trim.dig_xyz1) * 16384.0f / raw_rhall);
  retval = (process_comp_x0 - 16384.0f);
  process_comp_x1 = ((float)dev->trim.dig_xy2) * (retval * retval / 268435456.0f);
  process_comp_x2 = process_comp_x1 + retval * ((float)dev->trim.dig_xy1) / 16384.0f;
  process_comp_x3 = ((float)dev->trim.dig_x2) + 160.0f;
  process_comp_x4 = raw_x * ((process_comp_x2 + 256.0f) * process_comp_x3);
  retval = ((process_comp_x4 / 8192.0f) + (((float)dev->trim.dig_x1) * 8.0f)) / 16.0f;

  return retval;
}

/****************************************************************************
 * Name: bmm150_compensate_y
 ****************************************************************************/

static float bmm150_compensate_y(bmm150_dev_t *dev, int16_t raw_y, 
                                  uint16_t raw_rhall)
{
  float retval;
  float process_comp_y0;
  float process_comp_y1;
  float process_comp_y2;
  float process_comp_y3;
  float process_comp_y4;

  process_comp_y0 = ((float)dev->trim.dig_xyz1) * 16384.0f / raw_rhall;
  retval = process_comp_y0 - 16384.0f;
  process_comp_y1 = ((float)dev->trim.dig_xy2) * (retval * retval / 268435456.0f);
  process_comp_y2 = process_comp_y1 + retval * ((float)dev->trim.dig_xy1) / 16384.0f;
  process_comp_y3 = ((float)dev->trim.dig_y2) + 160.0f;
  process_comp_y4 = raw_y * (((process_comp_y2) + 256.0f) * process_comp_y3);
  retval = ((process_comp_y4 / 8192.0f) + (((float)dev->trim.dig_y1) * 8.0f)) / 16.0f;

  return retval;
}

/****************************************************************************
 * Name: bmm150_compensate_z
 ****************************************************************************/

static float bmm150_compensate_z(bmm150_dev_t *dev, int16_t raw_z, 
                                  uint16_t raw_rhall)
{
  float retval;
  float process_comp_z0;
  float process_comp_z1;
  float process_comp_z2;
  float process_comp_z3;
  float process_comp_z4;

  process_comp_z0 = ((float)raw_z) - ((float)dev->trim.dig_z4);
  process_comp_z1 = ((float)raw_rhall) - ((float)dev->trim.dig_xyz1);
  process_comp_z2 = (((float)dev->trim.dig_z3) * process_comp_z1);
  process_comp_z3 = ((float)dev->trim.dig_z1) * ((float)raw_rhall) / 32768.0f;
  process_comp_z4 = ((float)dev->trim.dig_z2) + process_comp_z3;
  retval = (process_comp_z0 * 131072.0f - process_comp_z2) / 
           ((process_comp_z4) * 4.0f) / 16.0f;

  return retval;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmm150_init
 ****************************************************************************/

int bmm150_init(bmm150_dev_t *dev, uint8_t bus, uint8_t addr)
{
  uint8_t chip_id;
  int ret;

  if (!dev)
    {
      return BMM150_ERR_INVALID;
    }

  memset(dev, 0, sizeof(bmm150_dev_t));

  /* Initialize I2C */

  ret = i2c_init(&dev->i2c, bus, addr);
  if (ret != I2C_OK)
    {
      _err("ERROR: Failed to initialize I2C: %d\n", ret);
      return BMM150_ERR_INIT;
    }

  usleep(BMM150_STARTUP_TIME_MS * 1000);

  /* Verify chip ID */

  ret = i2c_read_reg(&dev->i2c, BMM150_REG_CHIP_ID, &chip_id);
  if (ret != I2C_OK)
    {
      _err("ERROR: Failed to read chip ID\n");
      i2c_deinit(&dev->i2c);
      return BMM150_ERR_COMM;
    }

  if (chip_id != BMM150_CHIP_ID_VALUE)
    {
      _err("ERROR: Invalid chip ID: 0x%02X (expected 0x%02X)\n", 
           chip_id, BMM150_CHIP_ID_VALUE);
      i2c_deinit(&dev->i2c);
      return BMM150_ERR_INVALID;
    }

  _info("BMM150 detected, Chip ID: 0x%02X\n", chip_id);

  /* Soft reset */

  ret = bmm150_soft_reset(dev);
  if (ret != BMM150_OK)
    {
      _err("ERROR: Soft reset failed\n");
      i2c_deinit(&dev->i2c);
      return ret;
    }

  /* Power ON */

  ret = i2c_write_reg(&dev->i2c, BMM150_REG_PWR_CTRL, 
                      BMM150_PWR_CTRL_POWER_ON);
  if (ret != I2C_OK)
    {
      _err("ERROR: Power ON failed\n");
      i2c_deinit(&dev->i2c);
      return BMM150_ERR_COMM;
    }

  usleep(3000);  /* Wait for power-up */

  /* Read trim registers */

  ret = bmm150_read_trim_registers(dev);
  if (ret != BMM150_OK)
    {
      _err("ERROR: Failed to read trim data\n");
      i2c_deinit(&dev->i2c);
      return ret;
    }

  /* Set to Normal mode with Regular preset */

  ret = bmm150_set_preset_mode(dev, 1);  /* Regular mode */
  if (ret != BMM150_OK)
    {
      _err("ERROR: Failed to set preset mode\n");
      i2c_deinit(&dev->i2c);
      return ret;
    }

  /* Set to Normal operation mode with 10Hz ODR */

  ret = i2c_write_reg(&dev->i2c, BMM150_REG_OP_MODE, 
                      BMM150_OP_MODE_NORMAL | BMM150_ODR_10HZ);
  if (ret != I2C_OK)
    {
      _err("ERROR: Failed to set operation mode\n");
      i2c_deinit(&dev->i2c);
      return BMM150_ERR_COMM;
    }

  /* Initialize calibration to unity (no calibration) */

  dev->cal.offset[0] = 0.0f;
  dev->cal.offset[1] = 0.0f;
  dev->cal.offset[2] = 0.0f;
  dev->cal.scale[0] = 1.0f;
  dev->cal.scale[1] = 1.0f;
  dev->cal.scale[2] = 1.0f;
  dev->cal.valid = false;

  dev->initialized = true;
  dev->sample_count = 0;
  dev->error_count = 0;

  _info("BMM150 initialized successfully\n");

  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_deinit
 ****************************************************************************/

int bmm150_deinit(bmm150_dev_t *dev)
{
  if (!dev || !dev->initialized)
    {
      return BMM150_ERR_INVALID;
    }

  /* Set to sleep mode */

  i2c_write_reg(&dev->i2c, BMM150_REG_OP_MODE, BMM150_OP_MODE_SLEEP);

  /* Power OFF */

  i2c_write_reg(&dev->i2c, BMM150_REG_PWR_CTRL, BMM150_PWR_CTRL_POWER_OFF);

  /* Close I2C */

  i2c_deinit(&dev->i2c);

  dev->initialized = false;

  _info("BMM150 deinitialized\n");

  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_read_raw
 ****************************************************************************/

int bmm150_read_raw(bmm150_dev_t *dev, bmm150_raw_data_t *raw_data)
{
  uint8_t data[8];
  int ret;

  if (!dev || !dev->initialized || !raw_data)
    {
      return BMM150_ERR_INVALID;
    }

  /* Read all data registers (0x42 to 0x49) */

  ret = i2c_read_regs(&dev->i2c, BMM150_REG_DATA_X_LSB, data, 8);
  if (ret != I2C_OK)
    {
      dev->error_count++;
      return BMM150_ERR_COMM;
    }

  /* Parse raw data */

  raw_data->raw_x = (int16_t)((data[1] << 8) | data[0]) >> 3;
  raw_data->raw_y = (int16_t)((data[3] << 8) | data[2]) >> 3;
  raw_data->raw_z = (int16_t)((data[5] << 8) | data[4]) >> 1;
  raw_data->raw_rhall = (uint16_t)((data[7] << 8) | data[6]) >> 2;

  /* Check for data overflow */

  if (data[6] & 0x01)  /* Check RHALL overflow bit */
    {
      _warn("WARNING: Hall resistance overflow detected\n");
      return BMM150_ERR_OVERFLOW;
    }

  dev->sample_count++;

  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_read_mag
 ****************************************************************************/

int bmm150_read_mag(bmm150_dev_t *dev, bmm150_mag_data_t *mag_data)
{
  bmm150_raw_data_t raw;
  int ret;

  if (!dev || !dev->initialized || !mag_data)
    {
      return BMM150_ERR_INVALID;
    }

  /* Read raw data */

  ret = bmm150_read_raw(dev, &raw);
  if (ret != BMM150_OK)
    {
      return ret;
    }

  /* Apply compensation */

  mag_data->x = bmm150_compensate_x(dev, raw.raw_x, raw.raw_rhall);
  mag_data->y = bmm150_compensate_y(dev, raw.raw_y, raw.raw_rhall);
  mag_data->z = bmm150_compensate_z(dev, raw.raw_z, raw.raw_rhall);

  /* Apply user calibration if valid */

  if (dev->cal.valid)
    {
      mag_data->x = (mag_data->x - dev->cal.offset[0]) * dev->cal.scale[0];
      mag_data->y = (mag_data->y - dev->cal.offset[1]) * dev->cal.scale[1];
      mag_data->z = (mag_data->z - dev->cal.offset[2]) * dev->cal.scale[2];
    }

  /* Get timestamp */

  mag_data->timestamp = clock_systime_ticks() * (1000000 / CLOCKS_PER_SEC);

  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_set_preset_mode
 ****************************************************************************/

int bmm150_set_preset_mode(bmm150_dev_t *dev, uint8_t mode)
{
  uint8_t rep_xy, rep_z;
  int ret;

  if (!dev || !dev->initialized)
    {
      return BMM150_ERR_INVALID;
    }

  switch (mode)
    {
      case 0:  /* Low Power */
        rep_xy = BMM150_REP_XY_LOW_POWER;
        rep_z = BMM150_REP_Z_LOW_POWER;
        break;

      case 1:  /* Regular */
        rep_xy = BMM150_REP_XY_REGULAR;
        rep_z = BMM150_REP_Z_REGULAR;
        break;

      case 2:  /* Enhanced */
        rep_xy = BMM150_REP_XY_ENHANCED;
        rep_z = BMM150_REP_Z_ENHANCED;
        break;

      case 3:  /* High Accuracy */
        rep_xy = BMM150_REP_XY_HIGH_ACCURACY;
        rep_z = BMM150_REP_Z_HIGH_ACCURACY;
        break;

      default:
        return BMM150_ERR_INVALID;
    }

  ret = i2c_write_reg(&dev->i2c, BMM150_REG_REP_XY, rep_xy);
  if (ret != I2C_OK) return BMM150_ERR_COMM;

  ret = i2c_write_reg(&dev->i2c, BMM150_REG_REP_Z, rep_z);
  if (ret != I2C_OK) return BMM150_ERR_COMM;

  _info("BMM150 preset mode set to %d\n", mode);

  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_set_odr
 ****************************************************************************/

int bmm150_set_odr(bmm150_dev_t *dev, uint8_t odr)
{
  uint8_t reg_val;
  int ret;

  if (!dev || !dev->initialized)
    {
      return BMM150_ERR_INVALID;
    }

  /* Read current OP_MODE */

  ret = i2c_read_reg(&dev->i2c, BMM150_REG_OP_MODE, &reg_val);
  if (ret != I2C_OK) return BMM150_ERR_COMM;

  /* Clear ODR bits and set new value */

  reg_val = (reg_val & 0xC7) | (odr & 0x38);

  ret = i2c_write_reg(&dev->i2c, BMM150_REG_OP_MODE, reg_val);
  if (ret != I2C_OK) return BMM150_ERR_COMM;

  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_get_heading
 ****************************************************************************/

int bmm150_get_heading(bmm150_dev_t *dev, float *heading)
{
  bmm150_mag_data_t mag;
  int ret;

  if (!dev || !heading)
    {
      return BMM150_ERR_INVALID;
    }

  ret = bmm150_read_mag(dev, &mag);
  if (ret != BMM150_OK)
    {
      return ret;
    }

  /* Calculate heading (tilt-compensated would require accelerometer) */

  *heading = atan2f(mag.y, mag.x) * 180.0f / M_PI;

  if (*heading < 0)
    {
      *heading += 360.0f;
    }

  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_calibrate
 ****************************************************************************/

int bmm150_calibrate(bmm150_dev_t *dev, int num_samples)
{
  bmm150_mag_data_t mag;
  float min[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
  float max[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
  int i;
  int ret;

  if (!dev || !dev->initialized || num_samples <= 0)
    {
      return BMM150_ERR_INVALID;
    }

  _info("Starting magnetometer calibration (%d samples)...\n", num_samples);
  _info("Rotate the device in all directions!\n");

  /* Temporarily disable calibration */

  dev->cal.valid = false;

  /* Collect samples */

  for (i = 0; i < num_samples; i++)
    {
      ret = bmm150_read_mag(dev, &mag);
      if (ret != BMM150_OK)
        {
          continue;
        }

      /* Update min/max */

      if (mag.x < min[0]) min[0] = mag.x;
      if (mag.y < min[1]) min[1] = mag.y;
      if (mag.z < min[2]) min[2] = mag.z;

      if (mag.x > max[0]) max[0] = mag.x;
      if (mag.y > max[1]) max[1] = mag.y;
      if (mag.z > max[2]) max[2] = mag.z;

      usleep(50000);  /* 50ms delay between samples */

      if ((i % 20) == 0)
        {
          printf("Progress: %d/%d\n", i, num_samples);
        }
    }

  /* Calculate hard iron offset */

  dev->cal.offset[0] = (max[0] + min[0]) / 2.0f;
  dev->cal.offset[1] = (max[1] + min[1]) / 2.0f;
  dev->cal.offset[2] = (max[2] + min[2]) / 2.0f;

  /* Calculate soft iron scale */

  float avg_delta = ((max[0] - min[0]) + (max[1] - min[1]) + 
                     (max[2] - min[2])) / 3.0f;

  dev->cal.scale[0] = avg_delta / (max[0] - min[0]);
  dev->cal.scale[1] = avg_delta / (max[1] - min[1]);
  dev->cal.scale[2] = avg_delta / (max[2] - min[2]);

  dev->cal.valid = true;

  _info("Calibration complete!\n");
  _info("Offset: [%.2f, %.2f, %.2f]\n", 
        dev->cal.offset[0], dev->cal.offset[1], dev->cal.offset[2]);
  _info("Scale:  [%.3f, %.3f, %.3f]\n", 
        dev->cal.scale[0], dev->cal.scale[1], dev->cal.scale[2]);

  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_self_test
 ****************************************************************************/

int bmm150_self_test(bmm150_dev_t *dev)
{
  bmm150_mag_data_t mag;
  int ret;

  if (!dev || !dev->initialized)
    {
      return BMM150_ERR_INVALID;
    }

  _info("Running BMM150 self-test...\n");

  /* Read magnetometer data */

  ret = bmm150_read_mag(dev, &mag);
  if (ret != BMM150_OK)
    {
      _err("ERROR: Self-test failed - cannot read data\n");
      return ret;
    }

  /* Check if values are in reasonable range (±1300 µT for Earth's field) */

  if (fabsf(mag.x) > 1300.0f || fabsf(mag.y) > 1300.0f || 
      fabsf(mag.z) > 1300.0f)
    {
      _err("ERROR: Self-test failed - values out of range\n");
      _err("Mag: X=%.2f, Y=%.2f, Z=%.2f µT\n", mag.x, mag.y, mag.z);
      return BMM150_ERR_INVALID;
    }

  _info("Self-test PASSED\n");
  _info("Mag: X=%.2f, Y=%.2f, Z=%.2f µT\n", mag.x, mag.y, mag.z);

  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_print_status
 ****************************************************************************/

void bmm150_print_status(bmm150_dev_t *dev)
{
  if (!dev)
    {
      return;
    }

  printf("===== BMM150 Status =====\n");
  printf("Initialized:   %s\n", dev->initialized ? "YES" : "NO");
  printf("I2C Address:   0x%02X\n", dev->i2c.addr);
  printf("Sample Count:  %u\n", dev->sample_count);
  printf("Error Count:   %u\n", dev->error_count);
  printf("Calibrated:    %s\n", dev->cal.valid ? "YES" : "NO");
  
  if (dev->cal.valid)
    {
      printf("  Offset: [%.2f, %.2f, %.2f]\n",
             dev->cal.offset[0], dev->cal.offset[1], dev->cal.offset[2]);
      printf("  Scale:  [%.3f, %.3f, %.3f]\n",
             dev->cal.scale[0], dev->cal.scale[1], dev->cal.scale[2]);
    }

  printf("=========================\n");
}