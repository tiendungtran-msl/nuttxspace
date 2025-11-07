/****************************************************************************
 * apps/examples/icm42688p/icm42688p_driver.c
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <debug.h>
#include <nuttx/clock.h>

#include "icm42688p_driver.h"
#include "icm42688p_spi.h"
#include "icm42688p_registers.h"

/* Helper function to convert raw to scaled */
static void convert_accel_data(icm42688p_dev_t *dev, 
                               int16_t raw_x, int16_t raw_y, int16_t raw_z,
                               icm42688p_scaled_data_t *scaled)
{
    scaled->x = (float)raw_x / dev->accel_sensitivity;
    scaled->y = (float)raw_y / dev->accel_sensitivity;
    scaled->z = (float)raw_z / dev->accel_sensitivity;
}

static void convert_gyro_data(icm42688p_dev_t *dev,
                              int16_t raw_x, int16_t raw_y, int16_t raw_z,
                              icm42688p_scaled_data_t *scaled)
{
    scaled->x = ((float)raw_x / dev->gyro_sensitivity) - dev->gyro_offset.x;
    scaled->y = ((float)raw_y / dev->gyro_sensitivity) - dev->gyro_offset.y;
    scaled->z = ((float)raw_z / dev->gyro_sensitivity) - dev->gyro_offset.z;
}

static float convert_temperature(int16_t raw_temp)
{
    return ((float)raw_temp / TEMP_SENSITIVITY) + TEMP_OFFSET;
}

int icm42688p_initialize(icm42688p_dev_t *dev)
{
    uint8_t who_am_i;
    int ret;

    if (!dev) {
        return -EINVAL;
    }

    memset(dev, 0, sizeof(icm42688p_dev_t));

    /* Open SPI device */
    dev->spi_fd = icm42688p_spi_init(ICM42688P_SPI_BUS, ICM42688P_SPI_DEVID);
    if (dev->spi_fd < 0) {
        snerr("ERROR: Failed to initialize SPI\n");
        return dev->spi_fd;
    }

    /* Wait for power-on */
    usleep(100000);  /* 100ms */

    /* Verify WHO_AM_I */
    ret = icm42688p_select_bank(dev->spi_fd, BANK_0);
    if (ret < 0) {
        goto errout;
    }

    ret = icm42688p_read_register(dev->spi_fd, ICM42688P_WHO_AM_I, &who_am_i);
    if (ret < 0) {
        snerr("ERROR: Failed to read WHO_AM_I\n");
        goto errout;
    }

    if (who_am_i != ICM42688P_WHO_AM_I_VALUE) {
        snerr("ERROR: Invalid WHO_AM_I: 0x%02x (expected 0x47)\n", who_am_i);
        ret = -ENODEV;
        goto errout;
    }

    sninfo("ICM-42688-P detected (WHO_AM_I=0x%02x)\n", who_am_i);

    /* Reset device */
    ret = icm42688p_reset(dev);
    if (ret < 0) {
        goto errout;
    }

    /* Configure device */
    ret = icm42688p_configure(dev);
    if (ret < 0) {
        goto errout;
    }

    sninfo("ICM-42688-P initialized successfully\n");
    return OK;

errout:
    icm42688p_spi_deinit(dev->spi_fd);
    return ret;
}

int icm42688p_deinitialize(icm42688p_dev_t *dev)
{
    if (!dev) {
        return -EINVAL;
    }

    icm42688p_spi_deinit(dev->spi_fd);
    return OK;
}

int icm42688p_reset(icm42688p_dev_t *dev)
{
    int ret;

    /* Soft reset */
    ret = icm42688p_select_bank(dev->spi_fd, BANK_0);
    if (ret < 0) return ret;

    ret = icm42688p_write_register(dev->spi_fd, ICM42688P_DEVICE_CONFIG, 0x01);
    if (ret < 0) {
        snerr("ERROR: Soft reset failed\n");
        return ret;
    }

    /* Wait for reset to complete */
    usleep(100000);  /* 100ms */

    sninfo("ICM-42688-P reset complete\n");
    return OK;
}

int icm42688p_configure(icm42688p_dev_t *dev)
{
    int ret;

    /* Select Bank 0 */
    ret = icm42688p_select_bank(dev->spi_fd, BANK_0);
    if (ret < 0) return ret;

    /* Configure Power Management - Enable Gyro and Accel in Low Noise mode */
    ret = icm42688p_write_register(dev->spi_fd, ICM42688P_PWR_MGMT0,
                                    PWR_MGMT0_GYRO_MODE_LN | PWR_MGMT0_ACCEL_MODE_LN);
    if (ret < 0) {
        snerr("ERROR: Failed to configure power management\n");
        return ret;
    }

    usleep(50000);  /* 50ms for sensors to start */

    /* Configure Gyro: ±2000 dps, 1kHz ODR */
    dev->gyro_fs = GYRO_CONFIG0_FS_SEL_2000DPS;
    dev->gyro_sensitivity = GYRO_SENSITIVITY_2000DPS;
    
    ret = icm42688p_write_register(dev->spi_fd, ICM42688P_GYRO_CONFIG0,
                                    dev->gyro_fs | GYRO_CONFIG0_ODR_1KHZ);
    if (ret < 0) {
        snerr("ERROR: Failed to configure gyroscope\n");
        return ret;
    }

    /* Configure Accel: ±16g, 1kHz ODR */
    dev->accel_fs = ACCEL_CONFIG0_FS_SEL_16G;
    dev->accel_sensitivity = ACCEL_SENSITIVITY_16G;
    
    ret = icm42688p_write_register(dev->spi_fd, ICM42688P_ACCEL_CONFIG0,
                                    dev->accel_fs | ACCEL_CONFIG0_ODR_1KHZ);
    if (ret < 0) {
        snerr("ERROR: Failed to configure accelerometer\n");
        return ret;
    }

    /* Configure filter bandwidth (optional) */
    ret = icm42688p_write_register(dev->spi_fd, ICM42688P_GYRO_ACCEL_CONFIG0, 0x44);
    if (ret < 0) {
        snerr("ERROR: Failed to configure filters\n");
        return ret;
    }

    usleep(50000);  /* 50ms stabilization */

    sninfo("ICM-42688-P configured: Gyro ±2000dps, Accel ±16g, ODR 1kHz\n");
    return OK;
}

int icm42688p_read_data(icm42688p_dev_t *dev, icm42688p_data_t *data)
{
    uint8_t buffer[14];
    int16_t raw_temp, raw_accel_x, raw_accel_y, raw_accel_z;
    int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
    int ret;

    if (!dev || !data) {
        return -EINVAL;
    }

    /* Select Bank 0 */
    ret = icm42688p_select_bank(dev->spi_fd, BANK_0);
    if (ret < 0) return ret;

    /* Burst read all sensor data */
    ret = icm42688p_read_registers(dev->spi_fd, ICM42688P_TEMP_DATA1, buffer, 14);
    if (ret < 0) {
        dev->error_count++;
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
    convert_accel_data(dev, raw_accel_x, raw_accel_y, raw_accel_z, &data->accel);
    convert_gyro_data(dev, raw_gyro_x, raw_gyro_y, raw_gyro_z, &data->gyro);
    data->temperature = convert_temperature(raw_temp);
    data->timestamp = clock_systime_ticks() * (1000000 / CLK_TCK);

    dev->sample_count++;
    return OK;
}

int icm42688p_calibrate_gyro(icm42688p_dev_t *dev, int num_samples)
{
    icm42688p_data_t data;
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int i, ret;

    if (!dev || num_samples <= 0) {
        return -EINVAL;
    }

    sninfo("Starting gyro calibration with %d samples...\n", num_samples);
    sninfo("Keep sensor stationary!\n");

    usleep(2000000);  /* 2 seconds to stabilize */

    for (i = 0; i < num_samples; i++) {
        ret = icm42688p_read_data(dev, &data);
        if (ret < 0) {
            snerr("ERROR: Calibration read failed at sample %d\n", i);
            return ret;
        }

        /* Accumulate without offset correction */
        sum_x += (float)data.gyro.x + dev->gyro_offset.x;
        sum_y += (float)data.gyro.y + dev->gyro_offset.y;
        sum_z += (float)data.gyro.z + dev->gyro_offset.z;

        usleep(10000);  /* 10ms between samples */
    }

    /* Calculate average offset */
    dev->gyro_offset.x = sum_x / num_samples;
    dev->gyro_offset.y = sum_y / num_samples;
    dev->gyro_offset.z = sum_z / num_samples;

    sninfo("Gyro calibration complete:\n");
    sninfo("  Offset X: %.3f dps\n", dev->gyro_offset.x);
    sninfo("  Offset Y: %.3f dps\n", dev->gyro_offset.y);
    sninfo("  Offset Z: %.3f dps\n", dev->gyro_offset.z);

    return OK;
}

void icm42688p_print_status(icm42688p_dev_t *dev)
{
    if (!dev) {
        return;
    }

    printf("\n=== ICM-42688-P Status ===\n");
    printf("Samples read:  %lu\n", (unsigned long)dev->sample_count);
    printf("Errors:        %lu\n", (unsigned long)dev->error_count);
    printf("Gyro range:    ±2000 dps\n");
    printf("Accel range:   ±16g\n");
    printf("Gyro offset:   X=%.3f Y=%.3f Z=%.3f dps\n",
           dev->gyro_offset.x, dev->gyro_offset.y, dev->gyro_offset.z);
    printf("========================\n\n");
}