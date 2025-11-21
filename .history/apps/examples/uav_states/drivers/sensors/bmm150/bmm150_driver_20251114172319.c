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
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <math.h>
#include <sys/ioctl.h>

#include <nuttx/i2c/i2c_master.h>

#include "bmm150_driver.h"
#include "bmm150_regs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef M_PI
#  define M_PI 3.14159265358979323846f
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmm150_i2c_read
 ****************************************************************************/

static int bmm150_i2c_read(bmm150_dev_t *dev, uint8_t reg, 
                           uint8_t *buffer, size_t len)
{
  struct i2c_msg_s msg[2];
  int ret;

  if (!dev || !buffer || len == 0)
    {
      return -EINVAL;
    }

  /* Write register address */
  msg[0].frequency = 400000;
  msg[0].addr      = dev->i2c_addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  /* Read data */
  msg[1].frequency = 400000;
  msg[1].addr      = dev->i2c_addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buffer;
  msg[1].length    = len;

  ret = ioctl(dev->i2c_fd, I2CIOC_TRANSFER, (unsigned long)msg);
  if (ret < 0)
    {
      dev->error_count++;
      return ret;
    }

  return (ret == 2) ? OK : -EIO;
}

/****************************************************************************
 * Name: bmm150_i2c_write
 ****************************************************************************/

static int bmm150_i2c_write(bmm150_dev_t *dev, uint8_t reg, uint8_t value)
{
  struct i2c_msg_s msg;
  uint8_t buffer[2];
  int ret;

  if (!dev)
    {
      return -EINVAL;
    }

  buffer[0] = reg;
  buffer[1] = value;

  msg.frequency = 400000;
  msg.addr      = dev->i2c_addr;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = 2;

  ret = ioctl(dev->i2c_fd, I2CIOC_TRANSFER, (unsigned long)&msg);
  if (ret < 0)
    {
      dev->error_count++;
      return ret;
    }

  return (ret == 1) ? OK : -EIO;
}

/****************************************************************************
 * Name: bmm150_read_trim_data
 ****************************************************************************/

static int bmm150_read_trim_data(bmm150_dev_t *dev)
{
  uint8_t data[2];
  int ret;

  /* Read all trim registers */

  ret = bmm150_i2c_read(dev, BMM150_DIG_X1, 
                        (uint8_t *)&dev->trim.dig_x1, 1);
  if (ret < 0) return ret;

  ret = bmm150_i2c_read(dev, BMM150_DIG_Y1,
                        (uint8_t *)&dev->trim.dig_y1, 1);
  if (ret < 0) return ret;

  ret = bmm150_i2c_read(dev, BMM150_DIG_X2,
                        (uint8_t *)&dev->trim.dig_x2, 1);
  if (ret < 0) return ret;

  ret = bmm150_i2c_read(dev, BMM150_DIG_Y2,
                        (uint8_t *)&dev->trim.dig_y2, 1);
  if (ret < 0) return ret;

  ret = bmm150_i2c_read(dev, BMM150_DIG_XY1,
                        &dev->trim.dig_xy1, 1);
  if (ret < 0) return ret;

  ret = bmm150_i2c_read(dev, BMM150_DIG_XY2,
                        (uint8_t *)&dev->trim.dig_xy2, 1);
  if (ret < 0) return ret;

  /* Read 16-bit values */

  ret = bmm150_i2c_read(dev, BMM150_DIG_Z1_LSB, data, 2);
  if (ret < 0) return ret;
  dev->trim.dig_z1 = (uint16_t)(data[0] | (data[1] << 8));

  ret = bmm150_i2c_read(dev, BMM150_DIG_Z2_LSB, data, 2);
  if (ret < 0) return ret;
  dev->trim.dig_z2 = (int16_t)(data[0] | (data[1] << 8));

  ret = bmm150_i2c_read(dev, BMM150_DIG_Z3_LSB, data, 2);
  if (ret < 0) return ret;
  dev->trim.dig_z3 = (int16_t)(data[0] | (data[1] << 8));

  ret = bmm150_i2c_read(dev, BMM150_DIG_Z4_LSB, data, 2);
  if (ret < 0) return ret;
  dev->trim.dig_z4 = (int16_t)(data[0] | (data[1] << 8));

  ret = bmm150_i2c_read(dev, BMM150_DIG_XYZ1_LSB, data, 2);
  if (ret < 0) return ret;
  dev->trim.dig_xyz1 = (uint16_t)(data[0] | (data[1] << 8));

  syslog(LOG_INFO, "BMM150: Trim data loaded\n");

  return OK;
}

/****************************************************************************
 * Name: bmm150_compensate_x
 ****************************************************************************/

static float bmm150_compensate_x(bmm150_dev_t *dev, int16_t mag_x, 
                                 uint16_t rhall)
{
  float retval;
  float process_comp_x0;
  float process_comp_x1;
  float process_comp_x2;
  float process_comp_x3;
  float process_comp_x4;

  if (mag_x != -4096 && rhall != 0)
    {
      process_comp_x0 = ((float)dev->trim.dig_xyz1) * 16384.0f / rhall;
      retval = process_comp_x0 - 16384.0f;
      process_comp_x1 = ((float)dev->trim.dig_xy2) *
                        (retval * retval / 268435456.0f);
      process_comp_x2 = process_comp_x1 + retval *
                        ((float)dev->trim.dig_xy1) / 16384.0f;
      process_comp_x3 = ((float)dev->trim.dig_x2) + 160.0f;
      process_comp_x4 = mag_x * ((process_comp_x2 + 256.0f) *
                        process_comp_x3);
      retval = ((process_comp_x4 / 8192.0f) +
                (((float)dev->trim.dig_x1) * 8.0f)) / 16.0f;
    }
  else
    {
      retval = 0.0f;
    }

  return retval;
}

/****************************************************************************
 * Name: bmm150_compensate_y
 ****************************************************************************/

static float bmm150_compensate_y(bmm150_dev_t *dev, int16_t mag_y,
                                 uint16_t rhall)
{
  float retval;
  float process_comp_y0;
  float process_comp_y1;
  float process_comp_y2;
  float process_comp_y3;
  float process_comp_y4;

  if (mag_y != -4096 && rhall != 0)
    {
      process_comp_y0 = ((float)dev->trim.dig_xyz1) * 16384.0f / rhall;
      retval = process_comp_y0 - 16384.0f;
      process_comp_y1 = ((float)dev->trim.dig_xy2) *
                        (retval * retval / 268435456.0f);
      process_comp_y2 = process_comp_y1 + retval *
                        ((float)dev->trim.dig_xy1) / 16384.0f;
      process_comp_y3 = ((float)dev->trim.dig_y2) + 160.0f;
      process_comp_y4 = mag_y * (((process_comp_y2) + 256.0f) *
                        process_comp_y3);
      retval = ((process_comp_y4 / 8192.0f) +
                (((float)dev->trim.dig_y1) * 8.0f)) / 16.0f;
    }
  else
    {
      retval = 0.0f;
    }

  return retval;
}

/****************************************************************************
 * Name: bmm150_compensate_z
 ****************************************************************************/

static float bmm150_compensate_z(bmm150_dev_t *dev, int16_t mag_z,
                                 uint16_t rhall)
{
  float retval;
  float process_comp_z0;
  float process_comp_z1;
  float process_comp_z2;
  float process_comp_z3;
  float process_comp_z4;

  if (mag_z != -16384 && rhall != 0 && 
      dev->trim.dig_z2 != 0 && 
      dev->trim.dig_z1 != 0 &&
      dev->trim.dig_xyz1 != 0)
    {
      process_comp_z0 = ((float)mag_z) - ((float)dev->trim.dig_z4);
      process_comp_z1 = ((float)rhall) - ((float)dev->trim.dig_xyz1);
      process_comp_z2 = (((float)dev->trim.dig_z3) * process_comp_z1);
      process_comp_z3 = ((float)dev->trim.dig_z1) *
                        ((float)rhall) / 32768.0f;
      process_comp_z4 = ((float)dev->trim.dig_z2) + process_comp_z3;
      retval = (process_comp_z0 * 131072.0f - process_comp_z2) /
               ((process_comp_z4) * 4.0f) / 16.0f;
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

int bmm150_init(bmm150_dev_t *dev, int bus, uint8_t addr)
{
  char devpath[16];
  uint8_t chip_id;
  int ret;

  if (!dev)
    {
      return -EINVAL;
    }

  memset(dev, 0, sizeof(bmm150_dev_t));
  dev->i2c_addr = addr;

  /* Open I2C device */

  snprintf(devpath, sizeof(devpath), "/dev/i2c%d", bus);
  dev->i2c_fd = open(devpath, O_RDWR);
  if (dev->i2c_fd < 0)
    {
      syslog(LOG_ERR, "BMM150: Failed to open %s: %d\n", devpath, errno);
      return -errno;
    }

  syslog(LOG_INFO, "BMM150: Opened %s (addr 0x%02x)\n", devpath, addr);

  /* Read and verify chip ID */

  ret = bmm150_i2c_read(dev, BMM150_CHIP_ID, &chip_id, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BMM150: Failed to read chip ID: %d\n", ret);
      close(dev->i2c_fd);
      return ret;
    }

  if (chip_id != BMM150_CHIP_ID_VALUE)
    {
      syslog(LOG_ERR, "BMM150: Invalid chip ID: 0x%02x (expected 0x%02x)\n",
             chip_id, BMM150_CHIP_ID_VALUE);
      close(dev->i2c_fd);
      return -ENODEV;
    }

  syslog(LOG_INFO, "BMM150: Chip ID verified: 0x%02x\n", chip_id);

  /* Soft reset */

  ret = bmm150_i2c_write(dev, BMM150_PWR_CTRL, 
                         BMM150_PWR_CTRL_SOFT_RESET);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BMM150: Soft reset failed: %d\n", ret);
      close(dev->i2c_fd);
      return ret;
    }

  usleep(10000); /* 10ms reset delay */

  /* Power on */

  ret = bmm150_i2c_write(dev, BMM150_PWR_CTRL, BMM150_PWR_CTRL_POWER_ON);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BMM150: Power on failed: %d\n", ret);
      close(dev->i2c_fd);
      return ret;
    }

  usleep(5000); /* 5ms startup delay */

  /* Read trim data */

  ret = bmm150_read_trim_data(dev);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BMM150: Failed to read trim data: %d\n", ret);
      close(dev->i2c_fd);
      return ret;
    }

  /* Set to normal mode with 25Hz ODR */

  ret = bmm150_i2c_write(dev, BMM150_OP_MODE, 
                         BMM150_OP_MODE_NORMAL | BMM150_ODR_25HZ);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BMM150: Failed to set operation mode: %d\n", ret);
      close(dev->i2c_fd);
      return ret;
    }

  /* Set repetitions (regular preset) */

  ret = bmm150_i2c_write(dev, BMM150_REP_XY, BMM150_REGULAR_REPXY);
  if (ret < 0)
    {
      close(dev->i2c_fd);
      return ret;
    }

  ret = bmm150_i2c_write(dev, BMM150_REP_Z, BMM150_REGULAR_REPZ);
  if (ret < 0)
    {
      close(dev->i2c_fd);
      return ret;
    }

  usleep(10000); /* 10ms stabilization */

  /* Initialize calibration data */

  dev->cal.offset[0] = 0.0f;
  dev->cal.offset[1] = 0.0f;
  dev->cal.offset[2] = 0.0f;
  dev->cal.scale[0] = 1.0f;
  dev->cal.scale[1] = 1.0f;
  dev->cal.scale[2] = 1.0f;
  dev->cal.valid = false;

  syslog(LOG_INFO, "BMM150: Initialized successfully\n");

  return OK;
}

/****************************************************************************
 * Name: bmm150_deinit
 ****************************************************************************/

void bmm150_deinit(bmm150_dev_t *dev)
{
  if (!dev)
    {
      return;
    }

  if (dev->i2c_fd >= 0)
    {
      /* Put device to sleep */
      bmm150_i2c_write(dev, BMM150_OP_MODE, BMM150_OP_MODE_SLEEP);
      
      close(dev->i2c_fd);
      dev->i2c_fd = -1;

      syslog(LOG_INFO, "BMM150: Deinitialized (samples=%lu, errors=%lu)\n",
             (unsigned long)dev->sample_count, 
             (unsigned long)dev->error_count);
    }
}

/****************************************************************************
 * Name: bmm150_read
 ****************************************************************************/

int bmm150_read(bmm150_dev_t *dev, bmm150_data_t *data)
{
  uint8_t buffer[8];
  int16_t raw_x, raw_y, raw_z;
  uint16_t raw_rhall;
  int ret;

  if (!dev || !data)
    {
      return -EINVAL;
    }

  if (dev->i2c_fd < 0)
    {
      return -ENODEV;
    }

  /* Read all data registers (0x42-0x49) */

  ret = bmm150_i2c_read(dev, BMM150_DATA_X_LSB, buffer, 8);
  if (ret < 0)
    {
      return ret;
    }

  /* Extract 13-bit values for X, Y and 15-bit for Z */

  raw_x = (int16_t)(((int16_t)((int8_t)buffer[1])) * 32) |
          ((buffer[0] & 0xF8) >> 3);
  raw_y = (int16_t)(((int16_t)((int8_t)buffer[3])) * 32) |
          ((buffer[2] & 0xF8) >> 3);
  raw_z = (int16_t)(((int16_t)((int8_t)buffer[5])) * 128) |
          ((buffer[4] & 0xFE) >> 1);
  raw_rhall = (uint16_t)(((uint16_t)buffer[7] << 6) |
                         ((buffer[6] & 0xFC) >> 2));

  /* Compensate using trim data */

  data->x = bmm150_compensate_x(dev, raw_x, raw_rhall);
  data->y = bmm150_compensate_y(dev, raw_y, raw_rhall);
  data->z = bmm150_compensate_z(dev, raw_z, raw_rhall);

  /* Apply user calibration if available */

  if (dev->cal.valid)
    {
      data->x = (data->x - dev->cal.offset[0]) * dev->cal.scale[0];
      data->y = (data->y - dev->cal.offset[1]) * dev->cal.scale[1];
      data->z = (data->z - dev->cal.offset[2]) * dev->cal.scale[2];
    }

  /* Timestamp */

  data->timestamp = 0; /* TODO: Add proper timestamp */

  dev->sample_count++;

  return OK;
}

/****************************************************************************
 * Name: bmm150_calibrate
 ****************************************************************************/

int bmm150_calibrate(bmm150_dev_t *dev, int num_samples)
{
  bmm150_data_t data;
  float min[3] = {10000.0f, 10000.0f, 10000.0f};
  float max[3] = {-10000.0f, -10000.0f, -10000.0f};
  int i;
  int ret;

  if (!dev)
    {
      return -EINVAL;
    }

  if (num_samples <= 0)
    {
      num_samples = 500;
    }

  /* Reset calibration */

  dev->cal.offset[0] = 0.0f;
  dev->cal.offset[1] = 0.0f;
  dev->cal.offset[2] = 0.0f;
  dev->cal.scale[0] = 1.0f;
  dev->cal.scale[1] = 1.0f;
  dev->cal.scale[2] = 1.0f;
  dev->cal.valid = false;

  syslog(LOG_INFO, "BMM150: Starting calibration (%d samples)\n", 
         num_samples);
  syslog(LOG_INFO, "BMM150: Rotate sensor in all directions!\n");

  /* Collect samples */

  for (i = 0; i < num_samples; i++)
    {
      ret = bmm150_read(dev, &data);
      if (ret < 0)
        {
          syslog(LOG_ERR, "BMM150: Calibration read failed: %d\n", ret);
          return ret;
        }

      /* Track min/max for each axis */

      if (data.x < min[0]) min[0] = data.x;
      if (data.x > max[0]) max[0] = data.x;
      if (data.y < min[1]) min[1] = data.y;
      if (data.y > max[1]) max[1] = data.y;
      if (data.z < min[2]) min[2] = data.z;
      if (data.z > max[2]) max[2] = data.z;

      usleep(40000); /* 40ms = 25Hz */
    }

  /* Calculate hard iron offsets */

  dev->cal.offset[0] = (max[0] + min[0]) / 2.0f;
  dev->cal.offset[1] = (max[1] + min[1]) / 2.0f;
  dev->cal.offset[2] = (max[2] + min[2]) / 2.0f;

  /* Calculate soft iron scale factors */

  float avg_delta = ((max[0] - min[0]) + (max[1] - min[1]) +
                     (max[2] - min[2])) / 3.0f;

  if ((max[0] - min[0]) > 0.1f)
    dev->cal.scale[0] = avg_delta / (max[0] - min[0]);
  if ((max[1] - min[1]) > 0.1f)
    dev->cal.scale[1] = avg_delta / (max[1] - min[1]);
  if ((max[2] - min[2]) > 0.1f)
    dev->cal.scale[2] = avg_delta / (max[2] - min[2]);

  dev->cal.valid = true;

  syslog(LOG_INFO, "BMM150: Calibration complete\n");
  syslog(LOG_INFO, "  Offset: (%.2f, %.2f, %.2f) uT\n",
         dev->cal.offset[0], dev->cal.offset[1], dev->cal.offset[2]);
  syslog(LOG_INFO, "  Scale: (%.3f, %.3f, %.3f)\n",
         dev->cal.scale[0], dev->cal.scale[1], dev->cal.scale[2]);

  return OK;
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

  printf("BMM150 Status:\n");
  printf("  I2C Address: 0x%02x\n", dev->i2c_addr);
  printf("  Samples: %lu\n", (unsigned long)dev->sample_count);
  printf("  Errors: %lu\n", (unsigned long)dev->error_count);
  printf("  Calibration: %s\n", dev->cal.valid ? "Valid" : "Not calibrated");
  
  if (dev->cal.valid)
    {
      printf("    Offset: (%.2f, %.2f, %.2f) uT\n",
             dev->cal.offset[0], dev->cal.offset[1], dev->cal.offset[2]);
      printf("    Scale: (%.3f, %.3f, %.3f)\n",
             dev->cal.scale[0], dev->cal.scale[1], dev->cal.scale[2]);
    }
}