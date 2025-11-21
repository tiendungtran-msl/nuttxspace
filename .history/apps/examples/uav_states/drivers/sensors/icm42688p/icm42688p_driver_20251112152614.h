/****************************************************************************
 * apps/examples/imu_system/drivers/icm42688p/icm42688p_driver.h
 *
 * ICM-42688-P Multi-instance Driver
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_ICM42688P_DRIVER_H
#define __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_ICM42688P_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Raw sensor data */

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} icm42688p_raw_data_t;

/* Scaled sensor data */

typedef struct
{
  float x;
  float y;
  float z;
} icm42688p_scaled_data_t;

/* Complete sensor reading */

typedef struct
{
  icm42688p_scaled_data_t accel;      /* Acceleration in g */
  icm42688p_scaled_data_t gyro;       /* Angular velocity in dps */
  float temperature;                   /* Temperature in °C */
  uint64_t timestamp;                  /* Timestamp in microseconds */
} icm42688p_data_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: icm42688p_init
 *
 * Description:
 *   Initialize ICM42688P sensor instance
 *
 * Input Parameters:
 *   devid - Device ID (0-3 for the 4 sensors)
 *
 * Returned Value:
 *   0 on success, negative errno on failure
 *
 ****************************************************************************/

int icm42688p_init(uint8_t devid);

/****************************************************************************
 * Name: icm42688p_deinit
 *
 * Description:
 *   Deinitialize ICM42688P sensor instance
 *
 * Input Parameters:
 *   devid - Device ID
 *
 ****************************************************************************/

void icm42688p_deinit(uint8_t devid);

/****************************************************************************
 * Name: icm42688p_read
 *
 * Description:
 *   Read sensor data from ICM42688P
 *
 * Input Parameters:
 *   devid - Device ID
 *   data  - Pointer to data structure to fill
 *
 * Returned Value:
 *   0 on success, negative errno on failure
 *
 ****************************************************************************/

int icm42688p_read(uint8_t devid, icm42688p_data_t *data);

/****************************************************************************
 * Name: icm42688p_calibrate_gyro
 *
 * Description:
 *   Calibrate gyroscope by averaging samples while stationary
 *
 * Input Parameters:
 *   devid       - Device ID
 *   num_samples - Number of samples to average
 *
 * Returned Value:
 *   0 on success, negative errno on failure
 *
 ****************************************************************************/

int icm42688p_calibrate_gyro(uint8_t devid, int num_samples);

/****************************************************************************
 * Name: icm42688p_self_test
 *
 * Description:
 *   Run self-test on ICM42688P
 *
 * Input Parameters:
 *   devid - Device ID
 *
 * Returned Value:
 *   0 on success, negative errno on failure
 *
 ****************************************************************************/

int icm42688p_self_test(uint8_t devid);

#endif /* __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_ICM42688P_DRIVER_H */
