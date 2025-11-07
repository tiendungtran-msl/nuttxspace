/****************************************************************************
 * apps/examples/icm42688p/icm42688p_driver.h
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_ICM42688P_DRIVER_H
#define __APPS_EXAMPLES_ICM42688P_DRIVER_H

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/* Configuration */
#define ICM42688P_SPI_BUS               1
#define ICM42688P_SPI_DEVID             0
#define ICM42688P_SPI_MODE              SPIDEV_MODE3
#define ICM42688P_SPI_FREQUENCY         10000000  /* 10 MHz */

/* Data Structures */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} icm42688p_raw_data_t;

typedef struct {
    float x;
    float y;
    float z;
} icm42688p_scaled_data_t;

typedef struct {
    icm42688p_scaled_data_t accel;      /* g */
    icm42688p_scaled_data_t gyro;       /* dps */
    float temperature;                   /* °C */
    uint64_t timestamp;                  /* microseconds */
} icm42688p_data_t;

typedef struct {
    int spi_fd;
    
    uint8_t current_bank;
    
    /* Calibration */
    icm42688p_scaled_data_t gyro_offset;
    
    /* Configuration */
    uint8_t gyro_fs;
    uint8_t accel_fs;
    float gyro_sensitivity;
    float accel_sensitivity;
    
    /* Statistics */
    uint32_t sample_count;
    uint32_t error_count;
    
} icm42688p_dev_t;

/* Function Prototypes */
int icm42688p_initialize(icm42688p_dev_t *dev);
int icm42688p_deinitialize(icm42688p_dev_t *dev);
int icm42688p_reset(icm42688p_dev_t *dev);
int icm42688p_configure(icm42688p_dev_t *dev);
int icm42688p_read_data(icm42688p_dev_t *dev, icm42688p_data_t *data);
int icm42688p_read_fifo(icm42688p_dev_t *dev, icm42688p_data_t *data, 
                        int max_samples, int *samples_read);
int icm42688p_calibrate_gyro(icm42688p_dev_t *dev, int num_samples);
int icm42688p_self_test(icm42688p_dev_t *dev);
void icm42688p_print_status(icm42688p_dev_t *dev);

#endif /* __APPS_EXAMPLES_ICM42688P_DRIVER_H */