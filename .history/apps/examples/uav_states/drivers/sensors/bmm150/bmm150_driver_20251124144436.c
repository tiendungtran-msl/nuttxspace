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
#include <errno.h>
#include <syslog.h>
#include <math.h>
#include <nuttx/clock.h>
#include <nuttx/i2c/i2c_master.h>

#include "bmm150_driver.h"
#include "bmm150_regs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMM150_TIMEOUT_MS               100
#define BMM150_MAX_RETRIES              3

#define BMM150_I2C_FREQUENCY            100000 /* 100kHz */

/* Compensation formulas constants */

#define BMM150_XY_OVERFLOW_ADCVAL       -4096
#define BMM150_Z_OVERFLOW_ADCVAL        -16384
#define BMM150_OVERFLOW_OUTPUT_FLOAT    0.0f

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bmm150_write_reg(bmm150_dev_t *dev, uint8_t reg, uint8_t value);
static int bmm150_read_reg(bmm150_dev_t *dev, uint8_t reg, uint8_t *value);
static int bmm150_read_regs(bmm150_dev_t *dev, uint8_t reg, uint8_t *buffer,
                            uint8_t len);
static int bmm150_read_trim_registers(bmm150_dev_t *dev);
static int bmm150_soft_reset(bmm150_dev_t *dev);
static int bmm150_wait_soft_reset(bmm150_dev_t *dev);
static int bmm150_set_power_mode(bmm150_dev_t *dev, uint8_t mode);
static int bmm150_set_op_mode(bmm150_dev_t *dev, uint8_t mode);
static float bmm150_compensate_x(bmm150_dev_t *dev, int16_t mag_data_x,
                                  uint16_t data_rhall);
static float bmm150_compensate_y(bmm150_dev_t *dev, int16_t mag_data_y,
                                  uint16_t data_rhall);
static float bmm150_compensate_z(bmm150_dev_t *dev, int16_t mag_data_z,
                                  uint16_t data_rhall);
static void bmm150_apply_calibration(bmm150_dev_t *dev,
                                      bmm150_mag_data_t *mag_data);
static int bmm150_read_raw(bmm150_dev_t *dev, bmm150_raw_data_t *raw_data);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Ghi giá trị vào thanh ghi BMM150 */
static int bmm150_write_reg(bmm150_dev_t *dev, uint8_t reg, uint8_t value)
{
  int ret;
  struct i2c_config_s config;
  uint8_t buffer[2];

  if (!dev || !dev->initialized)
    {
      return BMM150_ERR_INIT;
    }
  
  /* Thiết lập cấu hình I2C */
  config.frequency = BMM150_I2C_FREQUENCY;
  config.address = dev->addr; 
  config.addrlen = 7; 

  buffer[0] = reg;
  buffer[1] = value;

  ret = i2c_write(dev->i2c, &config, buffer, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BMM150: I2C write reg 0x%02X failed: %d\n", reg, ret);
      dev->error_count++;
      return ret;
    }

  return ret;
}

/* Đọc giá trị từ thanh ghi BMM150 */
static int bmm150_read_reg(bmm150_dev_t *dev, uint8_t reg, uint8_t *value)
{
  uint8_t ret;
  struct i2c_config_s config;

  if (!dev || !dev->initialized || !value)
    {
      return BMM150_ERR_INIT;
    }

  /* Thiết lập cấu hình I2C */
  config.frequency = BMM150_I2C_FREQUENCY; /* 100kHz */
  config.address = dev->addr;
  config.addrlen = 7;

  ret = i2c_writeread(dev->i2c, &config, &reg, 1, value, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BMM150: I2C read reg 0x%02X failed: %d\n", reg, ret);
      dev->error_count++;
      return ret;
    }

  return ret;
}

/* Đọc nhiều giá trị từ các thanh ghi liên tiếp */
static int bmm150_read_regs(bmm150_dev_t *dev, uint8_t reg, uint8_t *buffer, uint8_t len)
{
  uint8_t ret;
  struct i2c_config_s config;

  if (!dev || !dev->initialized || !buffer)
    {
      return BMM150_ERR_INIT;
    }

  /* Thiết lập cấu hình I2C */
  config.frequency = BMM150_I2C_FREQUENCY; /* 100kHz */
  config.address = dev->addr;
  config.addrlen = 7;

  ret = i2c_writeread(dev->i2c, &config, &reg, 1, buffer, len);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BMM150: I2C read regs from 0x%02X failed: %d\n", reg, ret);
      dev->error_count++;
      return ret;
    }

  return ret;
}

static int bmm150_wait_soft_reset(bmm150_dev_t *dev)
{
  int ret;
  uint8_t reg_value;
  int timeout_ms = 0;

  /* Poll POWER_CONTROL register until reset bit is cleared */
  while (timeout_ms < BMM150_TIMEOUT_MS)
    {
      ret = bmm150_read_reg(dev, BMM150_REG_POWER_CONTROL, &reg_value);
      if (ret != BMM150_OK)
        {
          return ret;
        }

      if ((reg_value & BMM150_SET_SOFT_RESET) == 0)
        {
          return BMM150_OK; /* Reset completed */
        }

      usleep(1000); /* Chờ 1ms */
      timeout_ms++;
    }

  syslog(LOG_ERR, "BMM150: Soft reset timeout\n");
  return BMM150_ERR_COMM; /* Timeout */
}

/* Hàm soft reset */
static int bmm150_soft_reset(bmm150_dev_t *dev)
{
  int ret;

  ret = bmm150_write_reg(dev, BMM150_REG_POWER_CONTROL, BMM150_SET_SOFT_RESET);
  if (ret != BMM150_OK)
    {
      return ret;
    }

  usleep(BMM150_DELAY_SOFT_RESET * 1000); /* Chờ reset hoàn tất */

  return BMM150_OK;
}

/* Đọc các thanh ghi trim (điều chỉnh)*/
static int bmm150_read_trim_registers(bmm150_dev_t *dev)
{
  uint8_t trim_x1y1[2];
  uint8_t trim_xyz_data[4];
  uint8_t trim_xy1xy2[10];
  int ret;

  /* Read DIG_X1 and DIG_Y1 */
  ret = bmm150_read_regs(dev, BMM150_DIG_X1, trim_x1y1, 2);
  if (ret != BMM150_OK)
    {
      return ret;
    }

  dev->trim.dig_x1 = (int8_t)trim_x1y1[0];
  dev->trim.dig_y1 = (int8_t)trim_x1y1[1];

  /* Read DIG_Z4_LSB, DIG_Z4_MSB, DIG_X2, DIG_Y2 */
  ret = bmm150_read_regs(dev, BMM150_DIG_Z4_LSB, trim_xyz_data, 4);
  if (ret != BMM150_OK)
    {
      return ret;
    }

  dev->trim.dig_z4 = (int16_t)((uint16_t)((trim_xyz_data[1] << 8) | trim_xyz_data[0]));
  dev->trim.dig_x2 = (int8_t)trim_xyz_data[2];
  dev->trim.dig_y2 = (int8_t)trim_xyz_data[3];

  /* Read remaining trim data */

  ret = bmm150_read_regs(dev, BMM150_DIG_Z2_LSB, trim_xy1xy2, 10);
  if (ret != BMM150_OK)
    {
      return ret;
    }

  dev->trim.dig_z2    = (int16_t)((uint16_t)((trim_xy1xy2[1] << 8) | trim_xy1xy2[0]));
  dev->trim.dig_z1    = (uint16_t)((trim_xy1xy2[3] << 8) | trim_xy1xy2[2]);
  dev->trim.dig_xyz1  = (uint16_t)((trim_xy1xy2[5] << 8) | trim_xy1xy2[4]);
  dev->trim.dig_z3    = (int16_t)((uint16_t)((trim_xy1xy2[7] << 8) | trim_xy1xy2[6]));
  dev->trim.dig_xy2   = (int8_t)trim_xy1xy2[8];
  dev->trim.dig_xy1   = (uint8_t)trim_xy1xy2[9];

  return BMM150_OK;
}

/* Thiết lập chế độ nguồn điện */
static int bmm150_set_power_mode(bmm150_dev_t *dev, uint8_t mode)
{
  if(!dev)
  {
    syslog(LOG_ERR, "BMM150: Device handle is NULL\n");
    return BMM150_ERR_INIT;
  }

  return bmm150_write_reg(dev, BMM150_REG_POWER_CONTROL, mode);
}

/* Thiết lập chế độ hoạt động */
static int bmm150_set_op_mode(bmm150_dev_t *dev, uint8_t mode)
{
  int ret;
  uint8_t reg_data;

  ret = bmm150_read_reg(dev, BMM150_REG_OP_MODE, &reg_data);
  if (ret != BMM150_OK)
    {
      return ret;
    }

  reg_data = BMM150_SET_BITS(reg_data, BMM150_OP_MODE, mode);

  ret = bmm150_write_reg(dev, BMM150_REG_OP_MODE, reg_data);
  if (ret != BMM150_OK)
    {
      return ret;
    }

  usleep(BMM150_START_UP_TIME * 1000); /* Chờ chế độ hoạt động ổn định */
  return BMM150_OK;
}

/* Tiến hành bù trục X theo các tính toán của giá trị hiệu chỉnh */
static float bmm150_compensate_x(bmm150_dev_t *dev, int16_t mag_data_x, uint16_t data_rhall)
{
  float retval;
  float process_comp_x0;
  float process_comp_x1;
  float process_comp_x2;
  float process_comp_x3;
  float process_comp_x4;

  /* Overflow check */

  if (mag_data_x != BMM150_XY_OVERFLOW_ADCVAL)
    {
      if (data_rhall != 0)
        {
          /* Processing compensation equations */
          process_comp_x0 = (((float)dev->trim.dig_xyz1) * 16384.0f / data_rhall);
          retval = (process_comp_x0 - 16384.0f);
          process_comp_x1 = ((float)dev->trim.dig_xy2) * (retval * retval / 268435456.0f);
          process_comp_x2 = process_comp_x1 + retval * ((float)dev->trim.dig_xy1) / 16384.0f;
          process_comp_x3 = ((float)dev->trim.dig_x2) + 160.0f;
          process_comp_x4 = mag_data_x * ((process_comp_x2 + 256.0f) * process_comp_x3);
          retval = ((process_comp_x4 / 8192.0f) + (((float)dev->trim.dig_x1) * 8.0f)) / 16.0f;
        }
      else
        {
          retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
        }
    }
  else
    {
      retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
    }

  return retval;
}

/* Tiến hành bù trục Y theo các tính toán của giá trị hiệu chỉnh */
static float bmm150_compensate_y(bmm150_dev_t *dev, int16_t mag_data_y, uint16_t data_rhall)
{
  float retval;
  float process_comp_y0;
  float process_comp_y1;
  float process_comp_y2;
  float process_comp_y3;
  float process_comp_y4;

  /* Overflow check */

  if (mag_data_y != BMM150_XY_OVERFLOW_ADCVAL)
    {
      if (data_rhall != 0)
        {
          /* Processing compensation equations */

          process_comp_y0 = ((float)dev->trim.dig_xyz1) * 16384.0f / data_rhall;
          retval = process_comp_y0 - 16384.0f;
          process_comp_y1 = ((float)dev->trim.dig_xy2) * (retval * retval / 268435456.0f);
          process_comp_y2 = process_comp_y1 + retval * ((float)dev->trim.dig_xy1) / 16384.0f;
          process_comp_y3 = ((float)dev->trim.dig_y2) + 160.0f;
          process_comp_y4 = mag_data_y * (((process_comp_y2) + 256.0f) * process_comp_y3);
          retval = ((process_comp_y4 / 8192.0f) + (((float)dev->trim.dig_y1) * 8.0f)) / 16.0f;
        }
      else
        {
          retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
        }
    }
  else
    {
      retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
    }

  return retval;
}

/* Tiến hành bù trục Z theo các tính toán của giá trị hiệu chỉnh */
static float bmm150_compensate_z(bmm150_dev_t *dev, int16_t mag_data_z,
                                  uint16_t data_rhall)
{
  float retval;
  float process_comp_z0;
  float process_comp_z1;
  float process_comp_z2;
  float process_comp_z3;
  float process_comp_z4;

  /* Overflow check */

  if (mag_data_z != BMM150_Z_OVERFLOW_ADCVAL)
    {
      if ((dev->trim.dig_z2 != 0) && (dev->trim.dig_z1 != 0) &&
          (data_rhall != 0) && (dev->trim.dig_xyz1 != 0))
        {
          /* Processing compensation equations */
          process_comp_z0 = ((float)data_rhall) - ((float)dev->trim.dig_xyz1);
          process_comp_z1 = (((float)dev->trim.dig_z3) * process_comp_z0);
          process_comp_z2 = (((float)dev->trim.dig_z1) * ((float)data_rhall)) / 32768.0f;
          process_comp_z3 = (((float)dev->trim.dig_z2) * (process_comp_z0 * process_comp_z0 / 268435456.0f));
          process_comp_z4 = ((float)mag_data_z - process_comp_z1);
          retval = (process_comp_z4 * 131072.0f - process_comp_z2);
          retval = (retval / ((process_comp_z3 + 16384.0f) * 4.0f)) / 16.0f;
        }
      else
        {
          retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
        }
    }
  else
    {
      retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
    }

  return retval;
}

/* Áp dụng hiệu chỉnh */
static void bmm150_apply_calibration(bmm150_dev_t *dev, bmm150_mag_data_t *mag_data)
{
  if (!dev->cal.valid)
    {
      return;
    }

  /* Apply hard iron offset */
  mag_data->x -= dev->cal.offset[0];
  mag_data->y -= dev->cal.offset[1];
  mag_data->z -= dev->cal.offset[2];

  /* Apply soft iron scale */
  mag_data->x *= dev->cal.scale[0];
  mag_data->y *= dev->cal.scale[1];
  mag_data->z *= dev->cal.scale[2];
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmm150_init
 ****************************************************************************/

int bmm150_init(bmm150_dev_t *dev, uint8_t addr)
{
  int ret;
  uint8_t who_am_i;

  if (!dev)
    {
      return BMM150_ERR_INVALID;
    }

  memset(dev, 0, sizeof(bmm150_dev_t));
  dev->addr = addr;

  /* 1. Soft reset */
  ret = bmm150_soft_reset(dev);
  if (ret != BMM150_OK)
    {
      syslog(LOG_ERR, "BMM150: Soft reset failed\n");
      goto error;
    }
  usleep(BMM150_START_UP_TIME * 1000);

  /* 2. Read and verify chip address */
  ret = bmm150_read_reg(dev, addr, &who_am_i);
  if (ret != BMM150_OK)
    {
      syslog(LOG_ERR, "BMM150: Failed to read chip address\n");
      goto error;
    }

  if (who_am_i != addr)
    {
      syslog(LOG_ERR, "BMM150: Invalid chip address: 0x%02X (expected 0x%02X)\n", who_am_i, addr);
      ret = BMM150_ERR_INVALID;
      goto error;
    }
  syslog(LOG_INFO, "BMM150: Chip address verified: 0x%02X\n", who_am_i);

  /* 3. Power on */
  ret = bmm150_set_power_mode(dev, BMM150_POWER_CNTRL_ENABLE);
  if (ret != BMM150_OK)
    {
      syslog(LOG_ERR, "BMM150: Failed to power on\n");
      goto error;
    }
  syslog(LOG_INFO, "BMM150: Powered on\n");
  usleep(BMM150_START_UP_TIME * 1000);

  /* 4. Read trim registers */
  ret = bmm150_read_trim_registers(dev);
  if (ret != BMM150_OK)
    {
      syslog(LOG_ERR, "BMM150: Failed to read trim registers\n");
      goto error;
    }

  /* 5. Set to sleep mode initially */
  ret = bmm150_set_op_mode(dev, BMM150_POWERMODE_SLEEP);
  if (ret != BMM150_OK)
    {
      syslog(LOG_ERR, "BMM150: Failed to set sleep mode\n");
      goto error;
    }
  usleep(10 * 1000);

  /* 6. Set default preset mode (Regular) */
  ret = bmm150_set_preset_mode(dev, BMM150_PRESETMODE_REGULAR);
  if (ret != BMM150_OK)
    {
      syslog(LOG_ERR, "BMM150: Failed to set preset mode\n");
      goto error;
    }

  /* 7. Set to normal mode */
  ret = bmm150_set_op_mode(dev, BMM150_POWERMODE_NORMAL);
  if (ret != BMM150_OK)
    {
      syslog(LOG_ERR, "BMM150: Failed to set normal mode\n");
      goto error;
    }

  /* Initialize calibration data */
  dev->cal.valid = false;
  dev->initialized = true;
  syslog(LOG_INFO, "BMM150: Initialization successful\n");
  return BMM150_OK;

error:
  dev->initialized = false;
  bmm150_deinit(dev);
  return ret;
}

/****************************************************************************
 * Name: bmm150_deinit
 ****************************************************************************/

int bmm150_deinit(bmm150_dev_t *dev)
{
  if (!dev || !dev->initialized)
    {
      return BMM150_ERR_INIT;
    }

  /* Set to suspend mode */

  bmm150_set_op_mode(dev, BMM150_POWERMODE_SUSPEND);

  /* Power off */

  bmm150_set_power_mode(dev, BMM150_POWER_CNTRL_DISABLE);

  /* Deinitialize I2C */

  dev->initialized = false;

  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_read_raw
 ****************************************************************************/

static int bmm150_read_raw(bmm150_dev_t *dev, bmm150_raw_data_t *raw_data)
{
  uint8_t data[8];
  int ret;

  if (!dev || !dev->initialized || !raw_data)
    {
      return BMM150_ERR_INVALID;
    }

  /* Read magnetometer data registers (0x42 to 0x49) */

  ret = bmm150_read_regs(dev, BMM150_REG_DATA_X_LSB, data, 8);
  if (ret != BMM150_OK)
    {
      return ret;
    }

  raw_data->raw_x = (int16_t)(((int16_t)((int8_t)data[1]) << 5) | (data[0] >> 3));
  raw_data->raw_y = (int16_t)(((int16_t)((int8_t)data[3]) << 5) | (data[2] >> 3));
  raw_data->raw_z = (int16_t)(((int16_t)((int8_t)data[5]) << 7) | (data[4] >> 1));
  raw_data->raw_rhall = (uint16_t)(((uint16_t)data[7] << 6) | (data[6] >> 2));

  dev->sample_count++;

  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_read_mag
 ****************************************************************************/

int bmm150_read_mag(bmm150_dev_t *dev)
{
  bmm150_raw_data_t raw_data;
  int ret;

  if (!dev || !dev->initialized)
    {
      return BMM150_ERR_INVALID;
    }

  /* Read raw data */
  ret = bmm150_read_raw(dev, &raw_data);
  if (ret != BMM150_OK)
    {
      return ret;
    }

  /* Compensate magnetometer data using trim parameters */
  dev->bmm150_data.x = bmm150_compensate_x(dev, raw_data.raw_x, raw_data.raw_rhall);
  dev->bmm150_data.y = bmm150_compensate_y(dev, raw_data.raw_y, raw_data.raw_rhall);
  dev->bmm150_data.z = bmm150_compensate_z(dev, raw_data.raw_z, raw_data.raw_rhall);

  /* Apply user calibration if available */
  bmm150_apply_calibration(dev, &dev->bmm150_data);

  /* Get timestamp */
  dev->bmm150_data.timestamp = clock_systime_ticks() * USEC_PER_TICK;

  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_calibrate
 ****************************************************************************/

int bmm150_calibrate(bmm150_dev_t *dev, int num_samples)
{
  float min_x = 1000000.0f;
  float max_x = -1000000.0f;
  float min_y = 1000000.0f;
  float max_y = -1000000.0f;
  float min_z = 1000000.0f;
  float max_z = -1000000.0f;
  int ret;
  int i;

  if (!dev || !dev->initialized)
    {
      return BMM150_ERR_INVALID;
    }

  printf("BMM150: Starting calibration...\n");
  printf("Rotate sensor in all directions (figure-8 motion)\n");
  printf("Collecting %d samples...\n", num_samples);

  /* Temporarily disable calibration */

  bool cal_valid_backup = dev->cal.valid;
  dev->cal.valid = false;

  for (i = 0; i < num_samples; i++)
    {
      ret = bmm150_read_mag(dev);
      if (ret != BMM150_OK)
        {
          dev->cal.valid = cal_valid_backup;
          return ret;
        }

      /* Track min/max values for each axis */

      if (dev->bmm150_data.x < min_x)
        {
          min_x = dev->bmm150_data.x;
        }

      if (dev->bmm150_data.x > max_x)
        {
          max_x = dev->bmm150_data.x;
        }

      if (dev->bmm150_data.y < min_y)
        {
          min_y = dev->bmm150_data.y;
        }

      if (dev->bmm150_data.y > max_y)
        {
          max_y = dev->bmm150_data.y;
        }

      if (dev->bmm150_data.z < min_z)
        {
          min_z = dev->bmm150_data.z;
        }

      if (dev->bmm150_data.z > max_z)
        {
          max_z = dev->bmm150_data.z;
        }

      if (i % 100 == 0)
        {
          printf("  Progress: %d/%d\n", i, num_samples);
        }

      usleep(10 * 1000);
    }

  /* Calculate hard iron offset (center of ellipsoid) */

  dev->cal.offset[0] = (max_x + min_x) / 2.0f;
  dev->cal.offset[1] = (max_y + min_y) / 2.0f;
  dev->cal.offset[2] = (max_z + min_z) / 2.0f;

  /* Calculate soft iron scale (normalize to sphere) */

  float avg_delta = ((max_x - min_x) + (max_y - min_y) +
                    (max_z - min_z)) / 3.0f;

  dev->cal.scale[0] = avg_delta / (max_x - min_x);
  dev->cal.scale[1] = avg_delta / (max_y - min_y);
  dev->cal.scale[2] = avg_delta / (max_z - min_z);

  dev->cal.valid = true;

  printf("BMM150: Calibration complete!\n");
  printf("  Hard iron offset [X, Y, Z]: [%.2f, %.2f, %.2f] µT\n",
         dev->cal.offset[0], dev->cal.offset[1], dev->cal.offset[2]);
  printf("  Soft iron scale  [X, Y, Z]: [%.3f, %.3f, %.3f]\n",
         dev->cal.scale[0], dev->cal.scale[1], dev->cal.scale[2]);

  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_set_preset_mode
 ****************************************************************************/

int bmm150_set_preset_mode(bmm150_dev_t *dev, uint8_t mode)
{
  uint8_t rep_xy;
  uint8_t rep_z;
  uint8_t odr;
  int ret;

  if (!dev || !dev->initialized)
    {
      return BMM150_ERR_INVALID;
    }

  switch (mode)
    {
      case BMM150_PRESETMODE_LOWPOWER:
        rep_xy = BMM150_REPXY_LOWPOWER;
        rep_z = BMM150_REPZ_LOWPOWER;
        odr = BMM150_ODR_10HZ;
        break;

      case BMM150_PRESETMODE_REGULAR:
        rep_xy = BMM150_REPXY_REGULAR;
        rep_z = BMM150_REPZ_REGULAR;
        odr = BMM150_ODR_10HZ;
        break;

      case BMM150_PRESETMODE_ENHANCED:
        rep_xy = BMM150_REPXY_ENHANCED;
        rep_z = BMM150_REPZ_ENHANCED;
        odr = BMM150_ODR_10HZ;
        break;

      case BMM150_PRESETMODE_HIGHACCURACY:
        rep_xy = BMM150_REPXY_HIGHACCURACY;
        rep_z = BMM150_REPZ_HIGHACCURACY;
        odr = BMM150_ODR_20HZ;
        break;

      default:
        return BMM150_ERR_INVALID;
    }

  /* Set repetition for XY-axis */

  ret = bmm150_write_reg(dev, BMM150_REG_REP_XY, rep_xy);
  if (ret != BMM150_OK)
    {
      return ret;
    }

  /* Set repetition for Z-axis */

  ret = bmm150_write_reg(dev, BMM150_REG_REP_Z, rep_z);
  if (ret != BMM150_OK)
    {
      return ret;
    }

  /* Set output data rate */

  ret = bmm150_set_odr(dev, odr);
  if (ret != BMM150_OK)
    {
      return ret;
    }

  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_set_odr
 ****************************************************************************/

int bmm150_set_odr(bmm150_dev_t *dev, uint8_t odr)
{
  uint8_t reg_data;
  int ret;

  if (!dev || !dev->initialized)
    {
      return BMM150_ERR_INVALID;
    }

  ret = bmm150_read_reg(dev, BMM150_REG_OP_MODE, &reg_data);
  if (ret != BMM150_OK)
    {
      return ret;
    }

  reg_data = BMM150_SET_BITS_POS_0(reg_data, BMM150_ODR, odr);

  return bmm150_write_reg(dev, BMM150_REG_OP_MODE, reg_data);
}

/****************************************************************************
 * Name: bmm150_get_heading
 ****************************************************************************/

int bmm150_get_heading(bmm150_dev_t *dev)
{
  int ret;

  if (!dev || !dev->initialized)
    {
      return BMM150_ERR_INVALID;
    }

  ret = bmm150_read_mag(dev);
  if (ret != BMM150_OK)
    {
      return ret;
    }

  /* Calculate heading in degrees (0-360)
   * Note: This assumes the sensor is level (no tilt compensation)
   */

  dev->yaw = atan2f(dev->bmm150_data.y, dev->bmm150_data.x) * 180.0f / M_PI;

  /* Normalize to 0-360 */

  if (dev->yaw < 0)
    {
      dev->yaw += 360.0f;
    }

  return BMM150_OK;
}

/****************************************************************************
 * Name: bmm150_print_status
 ****************************************************************************/

void bmm150_print_status(bmm150_dev_t *dev)
{
  if (!dev)
    {
      printf("BMM150: Invalid device\n");
      return;
    }

  printf("===== BMM150 Status =====\n");
  printf("  Initialized:     %s\n", dev->initialized ? "Yes" : "No");
  printf("  I2C Address:     0x%02X\n", dev->addr);
  printf("  Sample Count:    %lu\n", dev->sample_count);
  printf("  Error Count:     %lu\n", dev->error_count);
  printf("  Calibration:     %s\n",
         dev->cal.valid ? "Valid" : "Not calibrated");

  if (dev->cal.valid)
    {
      printf("  Hard iron [µT]:  [%.2f, %.2f, %.2f]\n",
             dev->cal.offset[0], dev->cal.offset[1], dev->cal.offset[2]);
      printf("  Soft iron:       [%.3f, %.3f, %.3f]\n",
             dev->cal.scale[0], dev->cal.scale[1], dev->cal.scale[2]);
    }

  printf("========================\n");
}