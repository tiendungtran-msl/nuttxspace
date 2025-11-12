/****************************************************************************
 * apps/examples/imu_system/drivers/bmm150/bmm150_driver.h
 *
 * BMM150 Magnetometer Driver
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_BMM150_DRIVER_H
#define __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_BMM150_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Magnetometer data */

typedef struct
{
  float x;                  /* X-axis magnetic field in µT */
  float y;                  /* Y-axis magnetic field in µT */
  float z;                  /* Z-axis magnetic field in µT */
  uint64_t timestamp;       /* Timestamp in microseconds */
} bmm150_data_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bmm150_init
 *
 * Description:
 *   Initialize BMM150 magnetometer
 *
 * Returned Value:
 *   0 on success, negative errno on failure
 *
 ****************************************************************************/

int bmm150_init(void);

/****************************************************************************
 * Name: bmm150_deinit
 *
 * Description:
 *   Deinitialize BMM150 magnetometer
 *
 ****************************************************************************/

void bmm150_deinit(void);

/****************************************************************************
 * Name: bmm150_read
 *
 * Description:
 *   Read magnetometer data
 *
 * Input Parameters:
 *   data - Pointer to data structure to fill
 *
 * Returned Value:
 *   0 on success, negative errno on failure
 *
 ****************************************************************************/

int bmm150_read(bmm150_data_t *data);

/****************************************************************************
 * Name: bmm150_calibrate
 *
 * Description:
 *   Perform magnetometer calibration
 *
 * Returned Value:
 *   0 on success, negative errno on failure
 *
 ****************************************************************************/

int bmm150_calibrate(void);

#endif /* __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_BMM150_DRIVER_H */
