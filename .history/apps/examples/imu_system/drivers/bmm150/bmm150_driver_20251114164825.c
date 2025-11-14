/****************************************************************************
 * apps/examples/imu_system/drivers/bmm150/bmm150_driver.c
 *
 * BMM150 Magnetometer Driver Implementation (I2C Interface)
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
#include "../i2c/i2c_manager.h"
#include "../../utils/config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef M_PI
#  define M_PI 3.14159265358979323846f
#endif

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
  uint32_t read_count;
  uint32_t error_count;
} bmm150_dev_state_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bmm150_dev_state_t g_bmm150;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: read_trim_registers
 *
 * Description:
 *   Read factory trim/compensation data from BMM150
 ****************************************************************************/

static int read_trim_registers(void)
{
  uint8_t data[2];
  int ret;

  /* Read all trim registers */

  ret = i2c_manager_read_reg(IMU_BMM150_I2C_ADDR, BMM150_REG_DIG_X1,
                              (uint8_t *)&g_bmm150.trim.dig_x1);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read DIG_X1: %d\n", ret);
      return ret;
    }

  ret = i2c_manager_read_reg(IMU_BMM150_I2C_ADDR, BMM150_REG_DIG_Y1,
                              (uint8_t *)&g_bmm150.trim.dig_y1);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read DIG_Y1: %d\n", ret);
      return ret;
    }

  ret = i2c_manager_read_reg(IMU_BMM150_I2C_ADDR, BMM150_REG_DIG_X2,
                              (uint8_t *)&g_bmm150.trim.dig_x2);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read DIG_X2: %d\n", ret);
      return ret;
    }

  ret = i2c_manager_read_reg(IMU_BMM150_I2C_ADDR, BMM150_REG_DIG_Y2,
                              (uint8_t *)&g_bmm150.trim.dig_y2);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read DIG_Y2: %d\n", ret);
      return ret;
    }

  ret = i2c_manager_read_reg(IMU_BMM150_I2C_ADDR, BMM150_REG_DIG_XY1,
                              &g_bmm150.trim.dig_xy1);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read DIG_XY1: %d\n", ret);
      return ret;
    }

  ret = i2c_manager_read_reg(IMU_BMM150_I2C_ADDR, BMM150_REG_DIG_XY2,
                              (uint8_t *)&g_bmm150.trim.dig_xy2);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read DIG_XY2: %d\n", ret);
      return ret;
    }

  /* Read 16-bit trim values */

  ret = i2c_manager_read_regs(IMU_BMM150_I2C_ADDR, BMM150_REG_DIG_Z1_LSB,
                               data, 2);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read DIG_Z1: %d\n", ret);
      return ret;
    }
  g_bmm150.trim.dig_z1 = (uint16_t)(data[0] | (data[1] << 8));

  ret = i2c_manager_read_regs(IMU_BMM150_I2C_ADDR, BMM150_REG_DIG_Z2_LSB,
                               data, 2);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read DIG_Z2: %d\n", ret);
      return ret;
    }
  g_bmm150.trim.dig_z2 = (int16_t)(data[0] | (data[1] << 8));

  ret = i2c_manager_read_regs(IMU_BMM150_I2C_ADDR, BMM150_REG_DIG_Z3_LSB,
                               data, 2);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read DIG_Z3: %d\n", ret);
      return ret;
    }
  g_bmm150.trim.dig_z3 = (int16_t)(data[0] | (data[1] << 8));

  ret = i2c_manager_read_regs(IMU_BMM150_I2C_ADDR, BMM150_REG_DIG_Z4_LSB,
                               data, 2);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read DIG_Z4: %d\n", ret);
      return ret;
    }
  g_bmm150.trim.dig_z4 = (int16_t)(data[0] | (data[1] << 8));

  ret = i2c_manager_read_regs(IMU_BMM150_I2C_ADDR, BMM150_REG_DIG_XYZ1_LSB,
                               data, 2);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read DIG_XYZ1: %d\n", ret);
      return ret;
    }
  g_bmm150.trim.dig_xyz1 = (uint16_t)(data[0] | (data[1] << 8));

  sninfo("BMM150 trim data loaded successfully\n");
  sninfo("  dig_x1=%d, dig_y1=%d, dig_x2=%d, dig_y2=%d\n",
         g_bmm150.trim.dig_x1, g_bmm150.trim.dig_y1,
         g_bmm150.trim.dig_x2, g_bmm150.trim.dig_y2);

  return OK;
}

/****************************************************************************
 * Name: compensate_x
 *
 * Description:
 *   Apply temperature compensation for X-axis
 ****************************************************************************/

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

/****************************************************************************
 * Name: compensate_y
 *
 * Description:
 *   Apply temperature compensation for Y-axis
 ****************************************************************************/

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

/****************************************************************************
 * Name: compensate_z
 *
 * Description:
 *   Apply temperature compensation for Z-axis
 ****************************************************************************/

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
 *
 * Description:
 *   Initialize BMM150 magnetometer via I2C
 ****************************************************************************/

int bmm150_init(void)
{
  uint8_t chip_id;
  int ret;

  if (g_bmm150.initialized)
    {
      snwarn("WARNING: BMM150 already initialized\n");
      return -EBUSY;
    }

  memset(&g_bmm150, 0, sizeof(bmm150_dev_state_t));

  /* Default calibration values */

  g_bmm150.scale_x = 1.0f;
  g_bmm150.scale_y = 1.0f;
  g_bmm150.scale_z = 1.0f;

  sninfo("Initializing BMM150 on I2C%d at address 0x%02x\n",
         IMU_I2C_BUS, IMU_BMM150_I2C_ADDR);

  /* Read chip ID */

  ret = i2c_manager_read_reg(IMU_BMM150_I2C_ADDR, BMM150_REG_CHIP_ID,
                              &chip_id);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read BMM150 chip ID via I2C: %d\n", ret);
      snerr("       Check I2C connections (PB6=SCL, PB7=SDA)\n");
      g_bmm150.error_count++;
      return ret;
    }

  if (chip_id != BMM150_CHIP_ID)
    {
      snerr("ERROR: Invalid BMM150 chip ID: 0x%02x (expected 0x%02x)\n",
            chip_id, BMM150_CHIP_ID);
      g_bmm150.error_count++;
      return -ENODEV;
    }

  sninfo("BMM150 detected (CHIP_ID=0x%02x)\n", chip_id);

  /* Soft reset */

  ret = i2c_manager_write_reg(IMU_BMM150_I2C_ADDR, BMM150_REG_PWR_CTRL,
                               0x82);  /* Soft reset bit */
  if (ret < 0)
    {
      snerr("ERROR: Failed to reset BMM150\n");
      g_bmm150.error_count++;
      return ret;
    }

  usleep(10000);  /* 10ms reset time */

  /* Power on */

  ret = i2c_manager_write_reg(IMU_BMM150_I2C_ADDR, BMM150_REG_PWR_CTRL,
                               BMM150_PWR_CTRL_POWER_ON);
  if (ret < 0)
    {
      snerr("ERROR: Failed to power on BMM150\n");
      g_bmm150.error_count++;
      return ret;
    }

  usleep(5000);  /* 5ms startup time */

  /* Read trim registers */

  ret = read_trim_registers();
  if (ret < 0)
    {
      snerr("ERROR: Failed to read trim registers\n");
      g_bmm150.error_count++;
      return ret;
    }

  /* Set to normal mode with 25 Hz ODR */

  ret = i2c_manager_write_reg(IMU_BMM150_I2C_ADDR, BMM150_REG_OP_MODE,
                               BMM150_OP_MODE_NORMAL |
                               BMM150_DATA_RATE_25HZ);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set operation mode\n");
      g_bmm150.error_count++;
      return ret;
    }

  /* Set repetition for X/Y and Z */

  ret = i2c_manager_write_reg(IMU_BMM150_I2C_ADDR, BMM150_REG_REP_XY,
                               BMM150_REP_XY_REGULAR);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set XY repetition\n");
      g_bmm150.error_count++;
      return ret;
    }

  ret = i2c_manager_write_reg(IMU_BMM150_I2C_ADDR, BMM150_REG_REP_Z,
                               BMM150_REP_Z_REGULAR);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set Z repetition\n");
      g_bmm150.error_count++;
      return ret;
    }

  usleep(10000);  /* 10ms stabilization */

  g_bmm150.initialized = true;

  sninfo("BMM150 initialized successfully via I2C\n");
  sninfo("  Interface: I2C%d @ 400kHz\n", IMU_I2C_BUS);
  sninfo("  Address: 0x%02x\n", IMU_BMM150_I2C_ADDR);
  sninfo("  Mode: Normal, 25Hz ODR\n");

  return OK;
}

/****************************************************************************
 * Name: bmm150_deinit
 *
 * Description:
 *   Deinitialize BMM150 and put it to sleep mode
 ****************************************************************************/

void bmm150_deinit(void)
{
  if (g_bmm150.initialized)
    {
      /* Put device to sleep mode */

      i2c_manager_write_reg(IMU_BMM150_I2C_ADDR, BMM150_REG_PWR_CTRL,
                            BMM150_PWR_CTRL_POWER_OFF);

      sninfo("BMM150 deinitialized (reads=%u, errors=%u)\n",
             g_bmm150.read_count, g_bmm150.error_count);

      g_bmm150.initialized = false;
    }
}

/****************************************************************************
 * Name: bmm150_read
 *
 * Description:
 *   Read compensated magnetometer data
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
      snerr("ERROR: BMM150 not initialized\n");
      return -ENODEV;
    }

  /* Read all data registers (0x42-0x49) */

  ret = i2c_manager_read_regs(IMU_BMM150_I2C_ADDR, BMM150_REG_DATA_X_LSB,
                               buffer, 8);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read BMM150 data: %d\n", ret);
      g_bmm150.error_count++;
      return ret;
    }

  /* Extract 13-bit values for X, Y (bits 15-3) */
  /* Extract 15-bit value for Z (bits 15-1) */

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

  /* Apply user calibration (hard/soft iron) */

  data->x = (data->x - g_bmm150.offset_x) * g_bmm150.scale_x;
  data->y = (data->y - g_bmm150.offset_y) * g_bmm150.scale_y;
  data->z = (data->z - g_bmm150.offset_z) * g_bmm150.scale_z;

  /* Timestamp in microseconds */

  data->timestamp = clock_systime_ticks() * (1000000 / CLK_TCK);

  g_bmm150.read_count++;

  return OK;
}

/****************************************************************************
 * Name: bmm150_calibrate
 *
 * Description:
 *   Perform hard/soft iron calibration
 *   Requires rotating the sensor in all directions
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

  /* Reset calibration */

  g_bmm150.offset_x = 0.0f;
  g_bmm150.offset_y = 0.0f;
  g_bmm150.offset_z = 0.0f;
  g_bmm150.scale_x = 1.0f;
  g_bmm150.scale_y = 1.0f;
  g_bmm150.scale_z = 1.0f;

  printf("\n");
  printf("=================================================\n");
  printf("      BMM150 Magnetometer Calibration\n");
  printf("=================================================\n");
  printf("Collecting %d samples...\n", IMU_MAG_CALIB_SAMPLES);
  printf("ROTATE the sensor slowly in ALL DIRECTIONS!\n");
  printf("(Figure-8 motion works best)\n");
  printf("\n");

  /* Collect samples while rotating */

  for (i = 0; i < IMU_MAG_CALIB_SAMPLES; i++)
    {
      ret = bmm150_read(&data);
      if (ret < 0)
        {
          snerr("ERROR: Calibration read failed at sample %d\n", i);
          return ret;
        }

      /* Track min/max for each axis */

      if (data.x < min_x) min_x = data.x;
      if (data.x > max_x) max_x = data.x;
      if (data.y < min_y) min_y = data.y;
      if (data.y > max_y) max_y = data.y;
      if (data.z < min_z) min_z = data.z;
      if (data.z > max_z) max_z = data.z;

      /* Progress indicator */

      if ((i % 50) == 0)
        {
          printf("Progress: %3d%% [X: %6.1f-%6.1f, Y: %6.1f-%6.1f, "
                 "Z: %6.1f-%6.1f]\n",
                 (i * 100) / IMU_MAG_CALIB_SAMPLES,
                 min_x, max_x, min_y, max_y, min_z, max_z);
        }

      usleep(40000);  /* 40ms between samples (25 Hz) */
    }

  /* Calculate hard iron offsets (center point) */

  g_bmm150.offset_x = (max_x + min_x) / 2.0f;
  g_bmm150.offset_y = (max_y + min_y) / 2.0f;
  g_bmm150.offset_z = (max_z + min_z) / 2.0f;

  /* Calculate soft iron scale factors (normalize to sphere) */

  float avg_delta = ((max_x - min_x) + (max_y - min_y) +
                     (max_z - min_z)) / 3.0f;

  if ((max_x - min_x) > 0.1f)
    {
      g_bmm150.scale_x = avg_delta / (max_x - min_x);
    }

  if ((max_y - min_y) > 0.1f)
    {
      g_bmm150.scale_y = avg_delta / (max_y - min_y);
    }

  if ((max_z - min_z) > 0.1f)
    {
      g_bmm150.scale_z = avg_delta / (max_z - min_z);
    }

  printf("\n");
  printf("=================================================\n");
  printf("      Calibration Complete!\n");
  printf("=================================================\n");
  printf("Hard Iron Offsets (uT):\n");
  printf("  X: %7.2f\n", g_bmm150.offset_x);
  printf("  Y: %7.2f\n", g_bmm150.offset_y);
  printf("  Z: %7.2f\n", g_bmm150.offset_z);
  printf("\n");
  printf("Soft Iron Scale Factors:\n");
  printf("  X: %6.4f\n", g_bmm150.scale_x);
  printf("  Y: %6.4f\n", g_bmm150.scale_y);
  printf("  Z: %6.4f\n", g_bmm150.scale_z);
  printf("\n");
  printf("Field Range (uT):\n");
  printf("  X: %6.1f to %6.1f (delta: %6.1f)\n",
         min_x, max_x, max_x - min_x);
  printf("  Y: %6.1f to %6.1f (delta: %6.1f)\n",
         min_y, max_y, max_y - min_y);
  printf("  Z: %6.1f to %6.1f (delta: %6.1f)\n",
         min_z, max_z, max_z - min_z);
  printf("=================================================\n");
  printf("\n");

  return OK;
}

/****************************************************************************
 * Name: bmm150_self_test
 *
 * Description:
 *   Perform self-test (basic connectivity check)
 ****************************************************************************/

int bmm150_self_test(void)
{
  uint8_t chip_id;
  bmm150_data_t data;
  int ret;

  if (!g_bmm150.initialized)
    {
      return -ENODEV;
    }

  /* Test 1: Read chip ID */

  ret = i2c_manager_read_reg(IMU_BMM150_I2C_ADDR, BMM150_REG_CHIP_ID,
                              &chip_id);
  if (ret < 0 || chip_id != BMM150_CHIP_ID)
    {
      snerr("ERROR: Self-test failed - chip ID test\n");
      return -EIO;
    }

  /* Test 2: Read data */

  ret = bmm150_read(&data);
  if (ret < 0)
    {
      snerr("ERROR: Self-test failed - data read test\n");
      return ret;
    }

  /* Test 3: Check if data is reasonable (Earth's field: 25-65 uT) */

  float magnitude = sqrtf(data.x * data.x + data.y * data.y +
                          data.z * data.z);

  if (magnitude < 10.0f || magnitude > 100.0f)
    {
      snwarn("WARNING: Self-test - unusual field magnitude: %.1f uT\n",
             magnitude);
      snwarn("         Expected: 25-65 uT (Earth's magnetic field)\n");
    }

  sninfo("BMM150 self-test PASSED\n");
  sninfo("  Chip ID: 0x%02x\n", chip_id);
  sninfo("  Mag (X,Y,Z): (%.1f, %.1f, %.1f) uT\n", data.x, data.y, data.z);
  sninfo("  Magnitude: %.1f uT\n", magnitude);

  return OK;
}

/****************************************************************************
 * Name: bmm150_get_stats
 *
 * Description:
 *   Get driver statistics
 ****************************************************************************/

void bmm150_get_stats(uint32_t *reads, uint32_t *errors)
{
  if (reads)
    {
      *reads = g_bmm150.read_count;
    }

  if (errors)
    {
      *errors = g_bmm150.error_count;
    }
}