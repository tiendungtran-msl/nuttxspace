/****************************************************************************
 * apps/examples/imu_system/drivers/bmm150/bmm150_driver.c
 *
 * BMM150 Magnetometer Driver (I2C Interface)
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/clock.h>

#include "bmm150_driver.h"
#include "bmm150_regs.h"
#include "../i2c/i2c_manager.h" 

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int bmm150_i2c_write_reg(int fd, uint8_t addr, uint8_t reg, uint8_t value)
{
    struct i2c_msg_s msg[1];
    uint8_t buffer[2];
    int ret;

    buffer[0] = reg;
    buffer[1] = value;

    msg[0].frequency = 400000;  /* 400 kHz */
    msg[0].addr      = addr;
    msg[0].flags     = 0;
    msg[0].buffer    = buffer;
    msg[0].length    = 2;

    ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)msg);
    if (ret < 0)
    {
        snerr("ERROR: I2C write reg 0x%02x failed: %d\n", reg, errno);
        return -errno;
    }

    return OK;
}

static int bmm150_i2c_read_reg(int fd, uint8_t addr, uint8_t reg, uint8_t *value)
{
    struct i2c_msg_s msg[2];
    int ret;

    if (!value)
    {
        return -EINVAL;
    }

    msg[0].frequency = 400000;
    msg[0].addr      = addr;
    msg[0].flags     = 0;
    msg[0].buffer    = &reg;
    msg[0].length    = 1;

    msg[1].frequency = 400000;
    msg[1].addr      = addr;
    msg[1].flags     = I2C_M_READ;
    msg[1].buffer    = value;
    msg[1].length    = 1;

    ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)msg);
    if (ret < 0)
    {
        snerr("ERROR: I2C read reg 0x%02x failed: %d\n", reg, errno);
        return -errno;
    }

    return OK;
}

static int bmm150_i2c_read_regs(int fd, uint8_t addr, uint8_t reg, 
                                uint8_t *buffer, size_t len)
{
    struct i2c_msg_s msg[2];
    int ret;

    if (!buffer || len == 0)
    {
        return -EINVAL;
    }

    msg[0].frequency = 400000;
    msg[0].addr      = addr;
    msg[0].flags     = 0;
    msg[0].buffer    = &reg;
    msg[0].length    = 1;

    msg[1].frequency = 400000;
    msg[1].addr      = addr;
    msg[1].flags     = I2C_M_READ;
    msg[1].buffer    = buffer;
    msg[1].length    = len;

    ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)msg);
    if (ret < 0)
    {
        snerr("ERROR: I2C burst read from 0x%02x failed: %d\n", reg, errno);
        return -errno;
    }

    return OK;
}

static int bmm150_read_trim_data(bmm150_dev_t *dev)
{
    uint8_t data[2];
    int ret;

    /* Read trim registers */
    ret = bmm150_i2c_read_reg(dev->i2c_fd, dev->i2c_addr, 
                              BMM150_DIG_X1, (uint8_t *)&dev->trim.dig_x1);
    if (ret < 0) return ret;

    ret = bmm150_i2c_read_reg(dev->i2c_fd, dev->i2c_addr, 
                              BMM150_DIG_Y1, (uint8_t *)&dev->trim.dig_y1);
    if (ret < 0) return ret;

    ret = bmm150_i2c_read_reg(dev->i2c_fd, dev->i2c_addr, 
                              BMM150_DIG_X2, (uint8_t *)&dev->trim.dig_x2);
    if (ret < 0) return ret;

    ret = bmm150_i2c_read_reg(dev->i2c_fd, dev->i2c_addr, 
                              BMM150_DIG_Y2, (uint8_t *)&dev->trim.dig_y2);
    if (ret < 0) return ret;

    ret = bmm150_i2c_read_reg(dev->i2c_fd, dev->i2c_addr, 
                              BMM150_DIG_XY1, &dev->trim.dig_xy1);
    if (ret < 0) return ret;

    ret = bmm150_i2c_read_reg(dev->i2c_fd, dev->i2c_addr, 
                              BMM150_DIG_XY2, (uint8_t *)&dev->trim.dig_xy2);
    if (ret < 0) return ret;

    /* Read 16-bit trim values */
    ret = bmm150_i2c_read_regs(dev->i2c_fd, dev->i2c_addr, 
                               BMM150_DIG_Z1_LSB, data, 2);
    if (ret < 0) return ret;
    dev->trim.dig_z1 = (uint16_t)(data[0] | (data[1] << 8));

    ret = bmm150_i2c_read_regs(dev->i2c_fd, dev->i2c_addr, 
                               BMM150_DIG_Z2_LSB, data, 2);
    if (ret < 0) return ret;
    dev->trim.dig_z2 = (int16_t)(data[0] | (data[1] << 8));

    ret = bmm150_i2c_read_regs(dev->i2c_fd, dev->i2c_addr, 
                               BMM150_DIG_Z3_LSB, data, 2);
    if (ret < 0) return ret;
    dev->trim.dig_z3 = (int16_t)(data[0] | (data[1] << 8));

    ret = bmm150_i2c_read_regs(dev->i2c_fd, dev->i2c_addr, 
                               BMM150_DIG_Z4_LSB, data, 2);
    if (ret < 0) return ret;
    dev->trim.dig_z4 = (int16_t)(data[0] | (data[1] << 8));

    ret = bmm150_i2c_read_regs(dev->i2c_fd, dev->i2c_addr, 
                               BMM150_DIG_XYZ1_LSB, data, 2);
    if (ret < 0) return ret;
    dev->trim.dig_xyz1 = (uint16_t)(data[0] | (data[1] << 8));

    sninfo("BMM150 trim data loaded successfully\n");
    return OK;
}

static float bmm150_compensate_x(bmm150_dev_t *dev, int16_t raw_x, uint16_t raw_rhall)
{
    float retval;
    float process_comp_x0;
    float process_comp_x1;
    float process_comp_x2;
    float process_comp_x3;
    float process_comp_x4;

    if (raw_x != -4096)
    {
        if (raw_rhall != 0)
        {
            process_comp_x0 = (((float)dev->trim.dig_xyz1) * 16384.0f / raw_rhall);
            retval = (process_comp_x0 - 16384.0f);
        }
        else
        {
            retval = 0;
        }

        process_comp_x1 = ((float)dev->trim.dig_xy2) * (retval * retval / 268435456.0f);
        process_comp_x2 = process_comp_x1 + retval * ((float)dev->trim.dig_xy1) / 16384.0f;
        process_comp_x3 = ((float)dev->trim.dig_x2) + 160.0f;
        process_comp_x4 = raw_x * ((process_comp_x2 + 256.0f) * process_comp_x3);
        retval = ((process_comp_x4 / 8192.0f) + (((float)dev->trim.dig_x1) * 8.0f)) / 16.0f;
    }
    else
    {
        retval = -32768.0f;
    }

    return retval;
}

static float bmm150_compensate_y(bmm150_dev_t *dev, int16_t raw_y, uint16_t raw_rhall)
{
    float retval;
    float process_comp_y0;
    float process_comp_y1;
    float process_comp_y2;
    float process_comp_y3;
    float process_comp_y4;

    if (raw_y != -4096)
    {
        if (raw_rhall != 0)
        {
            process_comp_y0 = ((float)dev->trim.dig_xyz1) * 16384.0f / raw_rhall;
            retval = process_comp_y0 - 16384.0f;
        }
        else
        {
            retval = 0;
        }

        process_comp_y1 = ((float)dev->trim.dig_xy2) * (retval * retval / 268435456.0f);
        process_comp_y2 = process_comp_y1 + retval * ((float)dev->trim.dig_xy1) / 16384.0f;
        process_comp_y3 = ((float)dev->trim.dig_y2) + 160.0f;
        process_comp_y4 = raw_y * (((process_comp_y2) + 256.0f) * process_comp_y3);
        retval = ((process_comp_y4 / 8192.0f) + (((float)dev->trim.dig_y1) * 8.0f)) / 16.0f;
    }
    else
    {
        retval = -32768.0f;
    }

    return retval;
}

static float bmm150_compensate_z(bmm150_dev_t *dev, int16_t raw_z, uint16_t raw_rhall)
{
    float retval;
    float process_comp_z0;
    float process_comp_z1;
    float process_comp_z2;
    float process_comp_z3;
    float process_comp_z4;

    if (raw_z != -16384)
    {
        if (raw_rhall != 0)
        {
            process_comp_z0 = ((float)raw_z) - ((float)dev->trim.dig_z4);
            process_comp_z1 = ((float)raw_rhall) - ((float)dev->trim.dig_xyz1);
            process_comp_z2 = (((float)dev->trim.dig_z3) * process_comp_z1);
            process_comp_z3 = ((float)dev->trim.dig_z1) * ((float)raw_rhall) / 32768.0f;
            process_comp_z4 = ((float)dev->trim.dig_z2) + process_comp_z3;
            retval = (process_comp_z0 * 131072.0f - process_comp_z2) / 
                     ((process_comp_z4) * 4.0f);
        }
        else
        {
            retval = 0;
        }
    }
    else
    {
        retval = -32768.0f;
    }

    return retval;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bmm150_init(bmm150_dev_t *dev, int bus, uint8_t addr)
{
    char devpath[32];
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
        snerr("ERROR: Failed to open %s: %d\n", devpath, errno);
        return -errno;
    }

    sninfo("BMM150: Opened %s (fd=%d) at I2C addr 0x%02x\n", 
           devpath, dev->i2c_fd, dev->i2c_addr);

    usleep(10000);  /* 10ms startup time */

    /* Read and verify chip ID */
    ret = bmm150_i2c_read_reg(dev->i2c_fd, dev->i2c_addr, 
                              BMM150_CHIP_ID, &chip_id);
    if (ret < 0)
    {
        snerr("ERROR: Failed to read chip ID\n");
        goto errout;
    }

    if (chip_id != BMM150_CHIP_ID_VALUE)
    {
        snerr("ERROR: Invalid chip ID: 0x%02x (expected 0x%02x)\n", 
              chip_id, BMM150_CHIP_ID_VALUE);
        ret = -ENODEV;
        goto errout;
    }

    sninfo("BMM150 detected (chip_id=0x%02x)\n", chip_id);

    /* Soft reset */
    ret = bmm150_i2c_write_reg(dev->i2c_fd, dev->i2c_addr, 
                               BMM150_PWR_CTRL, BMM150_PWR_CTRL_SOFT_RESET);
    if (ret < 0)
    {
        snerr("ERROR: Soft reset failed\n");
        goto errout;
    }

    usleep(5000);  /* Wait for reset */

    /* Power on */
    ret = bmm150_i2c_write_reg(dev->i2c_fd, dev->i2c_addr, 
                               BMM150_PWR_CTRL, BMM150_PWR_CTRL_POWER_ON);
    if (ret < 0)
    {
        snerr("ERROR: Power on failed\n");
        goto errout;
    }

    usleep(3000);

    /* Read trim data */
    ret = bmm150_read_trim_data(dev);
    if (ret < 0)
    {
        snerr("ERROR: Failed to read trim data\n");
        goto errout;
    }

    /* Set repetitions for regular preset mode */
    ret = bmm150_i2c_write_reg(dev->i2c_fd, dev->i2c_addr, 
                               BMM150_REP_XY, BMM150_REGULAR_REPXY);
    if (ret < 0) goto errout;

    ret = bmm150_i2c_write_reg(dev->i2c_fd, dev->i2c_addr, 
                               BMM150_REP_Z, BMM150_REGULAR_REPZ);
    if (ret < 0) goto errout;

    /* Set normal mode with 25Hz ODR */
    ret = bmm150_i2c_write_reg(dev->i2c_fd, dev->i2c_addr, 
                               BMM150_OP_MODE, 
                               BMM150_OP_MODE_NORMAL | BMM150_ODR_25HZ);
    if (ret < 0)
    {
        snerr("ERROR: Failed to set operation mode\n");
        goto errout;
    }

    /* Initialize calibration to identity */
    dev->cal.offset[0] = 0.0f;
    dev->cal.offset[1] = 0.0f;
    dev->cal.offset[2] = 0.0f;
    dev->cal.scale[0] = 1.0f;
    dev->cal.scale[1] = 1.0f;
    dev->cal.scale[2] = 1.0f;
    dev->cal.valid = false;

    sninfo("BMM150 initialized successfully on I2C%d @ 0x%02x\n", bus, addr);
    return OK;

errout:
    close(dev->i2c_fd);
    dev->i2c_fd = -1;
    return ret;
}

void bmm150_deinit(bmm150_dev_t *dev)
{
    if (dev && dev->i2c_fd >= 0)
    {
        /* Set to suspend mode */
        bmm150_i2c_write_reg(dev->i2c_fd, dev->i2c_addr, 
                            BMM150_OP_MODE, BMM150_OP_MODE_SUSPEND);
        close(dev->i2c_fd);
        dev->i2c_fd = -1;
    }
}

int bmm150_read(bmm150_dev_t *dev, bmm150_data_t *data)
{
    uint8_t buffer[8];
    int16_t raw_x, raw_y, raw_z;
    uint16_t raw_rhall;
    float comp_x, comp_y, comp_z;
    int ret;

    if (!dev || !data)
    {
        return -EINVAL;
    }

    /* Read all mag data + hall resistance */
    ret = bmm150_i2c_read_regs(dev->i2c_fd, dev->i2c_addr, 
                               BMM150_DATA_X_LSB, buffer, 8);
    if (ret < 0)
    {
        dev->error_count++;
        return ret;
    }

    /* Parse raw data */
    raw_x = (int16_t)(((int16_t)((int8_t)buffer[1])) << 5) | (buffer[0] >> 3);
    raw_y = (int16_t)(((int16_t)((int8_t)buffer[3])) << 5) | (buffer[2] >> 3);
    raw_z = (int16_t)(((int16_t)((int8_t)buffer[5])) << 7) | (buffer[4] >> 1);
    raw_rhall = (uint16_t)(((uint16_t)buffer[7] << 6) | (buffer[6] >> 2));

    /* Compensate using trim data */
    comp_x = bmm150_compensate_x(dev, raw_x, raw_rhall);
    comp_y = bmm150_compensate_y(dev, raw_y, raw_rhall);
    comp_z = bmm150_compensate_z(dev, raw_z, raw_rhall);

    /* Apply calibration if available */
    if (dev->cal.valid)
    {
        data->x = (comp_x - dev->cal.offset[0]) * dev->cal.scale[0];
        data->y = (comp_y - dev->cal.offset[1]) * dev->cal.scale[1];
        data->z = (comp_z - dev->cal.offset[2]) * dev->cal.scale[2];
    }
    else
    {
        data->x = comp_x;
        data->y = comp_y;
        data->z = comp_z;
    }

    data->timestamp = clock_systime_ticks() * (1000000 / CLK_TCK);
    dev->sample_count++;

    return OK;
}

int bmm150_calibrate(bmm150_dev_t *dev, int num_samples)
{
    bmm150_data_t data;
    float min[3] = {1e9, 1e9, 1e9};
    float max[3] = {-1e9, -1e9, -1e9};
    int i, ret;

    if (!dev || num_samples <= 0)
    {
        return -EINVAL;
    }

    sninfo("Starting magnetometer calibration with %d samples\n", num_samples);
    sninfo("Rotate sensor slowly in all directions!\n");

    /* Temporarily disable calibration */
    dev->cal.valid = false;

    for (i = 0; i < num_samples; i++)
    {
        ret = bmm150_read(dev, &data);
        if (ret < 0)
        {
            snerr("ERROR: Calibration read failed at sample %d\n", i);
            return ret;
        }

        /* Track min/max for each axis */
        if (data.x < min[0]) min[0] = data.x;
        if (data.y < min[1]) min[1] = data.y;
        if (data.z < min[2]) min[2] = data.z;
        if (data.x > max[0]) max[0] = data.x;
        if (data.y > max[1]) max[1] = data.y;
        if (data.z > max[2]) max[2] = data.z;

        usleep(50000);  /* 50ms between samples */

        if (i % 100 == 0)
        {
            sninfo("  Progress: %d/%d samples\n", i, num_samples);
        }
    }

    /* Calculate hard iron offset (midpoint) */
    dev->cal.offset[0] = (max[0] + min[0]) / 2.0f;
    dev->cal.offset[1] = (max[1] + min[1]) / 2.0f;
    dev->cal.offset[2] = (max[2] + min[2]) / 2.0f;

    /* Calculate soft iron scale (normalize to average) */
    float avg_delta = ((max[0] - min[0]) + (max[1] - min[1]) + (max[2] - min[2])) / 3.0f;
    dev->cal.scale[0] = avg_delta / (max[0] - min[0]);
    dev->cal.scale[1] = avg_delta / (max[1] - min[1]);
    dev->cal.scale[2] = avg_delta / (max[2] - min[2]);

    dev->cal.valid = true;

    sninfo("Magnetometer calibration complete:\n");
    sninfo("  Hard iron offset: X=%.2f Y=%.2f Z=%.2f uT\n",
           dev->cal.offset[0], dev->cal.offset[1], dev->cal.offset[2]);
    sninfo("  Soft iron scale:  X=%.3f Y=%.3f Z=%.3f\n",
           dev->cal.scale[0], dev->cal.scale[1], dev->cal.scale[2]);

    return OK;
}

void bmm150_print_status(bmm150_dev_t *dev)
{
    if (!dev)
    {
        return;
    }

    printf("\n=== BMM150 Status ===\n");
    printf("I2C Address:   0x%02x\n", dev->i2c_addr);
    printf("Samples read:  %lu\n", (unsigned long)dev->sample_count);
    printf("Errors:        %lu\n", (unsigned long)dev->error_count);
    printf("Calibration:   %s\n", dev->cal.valid ? "Valid" : "Not calibrated");
    if (dev->cal.valid)
    {
        printf("  Hard iron: X=%.2f Y=%.2f Z=%.2f uT\n",
               dev->cal.offset[0], dev->cal.offset[1], dev->cal.offset[2]);
        printf("  Soft iron: X=%.3f Y=%.3f Z=%.3f\n",
               dev->cal.scale[0], dev->cal.scale[1], dev->cal.scale[2]);
    }
    printf("====================\n\n");
}