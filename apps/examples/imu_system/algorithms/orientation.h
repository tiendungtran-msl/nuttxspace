/****************************************************************************
 * apps/examples/imu_system/algorithms/orientation.h
 *
 * Orientation Calculation with Tilt Compensation
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_IMU_SYSTEM_ALGORITHMS_ORIENTATION_H
#define __APPS_EXAMPLES_IMU_SYSTEM_ALGORITHMS_ORIENTATION_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: orientation_init
 *
 * Description:
 *   Initialize orientation calculation
 *
 ****************************************************************************/

void orientation_init(void);

/****************************************************************************
 * Name: orientation_calculate
 *
 * Description:
 *   Calculate heading with tilt compensation
 *
 * Input Parameters:
 *   roll  - Roll angle in radians
 *   pitch - Pitch angle in radians
 *   mx    - Magnetometer X reading in µT
 *   my    - Magnetometer Y reading in µT
 *   mz    - Magnetometer Z reading in µT
 *
 * Returned Value:
 *   Heading in radians (0 = North, π/2 = East, π = South, -π/2 = West)
 *
 ****************************************************************************/

float orientation_calculate(float roll, float pitch,
                            float mx, float my, float mz);

/****************************************************************************
 * Name: orientation_get_heading
 *
 * Description:
 *   Get current heading in degrees (0-360)
 *
 * Returned Value:
 *   Heading in degrees
 *
 ****************************************************************************/

float orientation_get_heading(void);

#endif /* __APPS_EXAMPLES_IMU_SYSTEM_ALGORITHMS_ORIENTATION_H */
