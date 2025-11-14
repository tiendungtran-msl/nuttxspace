/****************************************************************************
 * apps/examples/imu_system/algorithms/sensor_fusion.h
 *
 * Madgwick AHRS Algorithm for Sensor Fusion
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_IMU_SYSTEM_ALGORITHMS_SENSOR_FUSION_H
#define __APPS_EXAMPLES_IMU_SYSTEM_ALGORITHMS_SENSOR_FUSION_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Quaternion representation */

typedef struct
{
  float w;
  float x;
  float y;
  float z;
} quaternion_t;

/* Euler angles in radians */

typedef struct
{
  float roll;
  float pitch;
  float yaw;
} euler_angles_t;

/* Fusion result */

typedef struct
{
  quaternion_t quaternion;
  euler_angles_t euler;
  uint64_t timestamp;
} fusion_result_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: fusion_init
 *
 * Description:
 *   Initialize sensor fusion algorithm
 *
 * Input Parameters:
 *   beta        - Filter gain (typically 0.033 to 0.1)
 *   sample_freq - Update rate in Hz
 *
 ****************************************************************************/

void fusion_init(float beta, float sample_freq);

/****************************************************************************
 * Name: fusion_update
 *
 * Description:
 *   Update fusion with new IMU and magnetometer data
 *
 * Input Parameters:
 *   gx, gy, gz - Gyroscope readings in rad/s
 *   ax, ay, az - Accelerometer readings in g
 *   mx, my, mz - Magnetometer readings in µT
 *
 ****************************************************************************/

void fusion_update(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float mx, float my, float mz);

/****************************************************************************
 * Name: fusion_get_quaternion
 *
 * Description:
 *   Get current orientation as quaternion
 *
 * Input Parameters:
 *   q - Pointer to quaternion structure to fill
 *
 ****************************************************************************/

void fusion_get_quaternion(quaternion_t *q);

/****************************************************************************
 * Name: fusion_get_euler
 *
 * Description:
 *   Get current orientation as Euler angles
 *
 * Input Parameters:
 *   euler - Pointer to Euler angles structure to fill
 *
 ****************************************************************************/

void fusion_get_euler(euler_angles_t *euler);

/****************************************************************************
 * Name: fusion_get_result
 *
 * Description:
 *   Get complete fusion result
 *
 * Input Parameters:
 *   result - Pointer to fusion result structure to fill
 *
 ****************************************************************************/

void fusion_get_result(fusion_result_t *result);

#endif /* __APPS_EXAMPLES_IMU_SYSTEM_ALGORITHMS_SENSOR_FUSION_H */
