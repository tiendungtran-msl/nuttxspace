/****************************************************************************
 * apps/examples/imu_system/drivers/bmm150/bmm150_driver.h
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_IMU_SYSTEM_BMM150_DRIVER_H
#define __APPS_EXAMPLES_IMU_SYSTEM_BMM150_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Trim data structure */
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

/* Magnetometer data */
typedef struct
{
    float x;        /* uT */
    float y;        /* uT */
    float z;        /* uT */
    uint64_t timestamp;
} bmm150_data_t;

/* Calibration data */
typedef struct
{
    float offset[3];     /* Hard iron offset */
    float scale[3];      /* Soft iron scale */
    bool valid;
} bmm150_cal_data_t;

/* Device structure */
typedef struct
{
    int i2c_fd;                      /* I2C file descriptor */
    uint8_t i2c_addr;                /* I2C device address (7-bit) */
    bmm150_trim_data_t trim;         /* Factory trim data */
    bmm150_cal_data_t cal;           /* Calibration data */
    uint32_t sample_count;
    uint32_t error_count;
} bmm150_dev_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize BMM150 on I2C bus
 * Parameters:
 *   dev  - Device structure
 *   bus  - I2C bus number (e.g., 1 for /dev/i2c1)
 *   addr - I2C address (BMM150_I2C_ADDR_PRIMARY or SECONDARY)
 */
int bmm150_init(bmm150_dev_t *dev, int bus, uint8_t addr);

/* Deinitialize and close I2C device */
void bmm150_deinit(bmm150_dev_t *dev);

/* Read magnetometer data */
int bmm150_read(bmm150_dev_t *dev, bmm150_data_t *data);

/* Calibrate magnetometer (hard/soft iron) */
int bmm150_calibrate(bmm150_dev_t *dev, int num_samples);

/* Print device status */
void bmm150_print_status(bmm150_dev_t *dev);

#endif /* __APPS_EXAMPLES_IMU_SYSTEM_BMM150_DRIVER_H */