/****************************************************************************
 * apps/examples/imu_system/algorithms/madgwick.h
 *
 * Madgwick AHRS Algorithm Header
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_IMU_SYSTEM_ALGORITHMS_MADGWICK_H
#define __APPS_EXAMPLES_IMU_SYSTEM_ALGORITHMS_MADGWICK_H

#include <stdint.h>

void madgwick_init(float beta);
void madgwick_update_imu(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt);
void madgwick_update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz,
                     float dt);
void madgwick_get_quaternion(float *q0, float *q1, float *q2, float *q3);

#endif