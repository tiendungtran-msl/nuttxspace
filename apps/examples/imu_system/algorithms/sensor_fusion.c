/****************************************************************************
 * apps/examples/imu_system/algorithms/sensor_fusion.c
 *
 * Madgwick AHRS Algorithm Implementation
 * Based on Sebastian Madgwick's open-source algorithm
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <math.h>
#include <nuttx/clock.h>

#include "sensor_fusion.h"
#include "../utils/config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef M_PI
#  define M_PI 3.14159265358979323846f
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static quaternion_t g_q = {1.0f, 0.0f, 0.0f, 0.0f};
static float g_beta = IMU_MADGWICK_BETA;
static float g_sample_period = 1.0f / IMU_MADGWICK_SAMPLE_FREQ;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static float inv_sqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fusion_init
 ****************************************************************************/

void fusion_init(float beta, float sample_freq)
{
  g_beta = beta;
  g_sample_period = 1.0f / sample_freq;
  g_q.w = 1.0f;
  g_q.x = 0.0f;
  g_q.y = 0.0f;
  g_q.z = 0.0f;
}

/****************************************************************************
 * Name: fusion_update
 ****************************************************************************/

void fusion_update(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float mx, float my, float mz)
{
  float recipnorm;
  float s0;
  float s1;
  float s2;
  float s3;
  float qDot1;
  float qDot2;
  float qDot3;
  float qDot4;
  float hx;
  float hy;
  float _2q0mx;
  float _2q0my;
  float _2q0mz;
  float _2q1mx;
  float _2bx;
  float _2bz;
  float _4bx;
  float _4bz;
  float _2q0;
  float _2q1;
  float _2q2;
  float _2q3;
  float _2q0q2;
  float _2q2q3;
  float q0q0;
  float q0q1;
  float q0q2;
  float q0q3;
  float q1q1;
  float q1q2;
  float q1q3;
  float q2q2;
  float q2q3;
  float q3q3;

  /* Use IMU algorithm if magnetometer measurement invalid */

  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
      /* Rate of change of quaternion from gyroscope */

      qDot1 = 0.5f * (-g_q.x * gx - g_q.y * gy - g_q.z * gz);
      qDot2 = 0.5f * (g_q.w * gx + g_q.y * gz - g_q.z * gy);
      qDot3 = 0.5f * (g_q.w * gy - g_q.x * gz + g_q.z * gx);
      qDot4 = 0.5f * (g_q.w * gz + g_q.x * gy - g_q.y * gx);

      /* Compute feedback only if accelerometer valid */

      if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
        {
          /* Normalize accelerometer measurement */

          recipnorm = inv_sqrt(ax * ax + ay * ay + az * az);
          ax *= recipnorm;
          ay *= recipnorm;
          az *= recipnorm;

          /* Auxiliary variables to avoid repeated arithmetic */

          _2q0 = 2.0f * g_q.w;
          _2q1 = 2.0f * g_q.x;
          _2q2 = 2.0f * g_q.y;
          _2q3 = 2.0f * g_q.z;
          q0q0 = g_q.w * g_q.w;
          q1q1 = g_q.x * g_q.x;
          q2q2 = g_q.y * g_q.y;
          q3q3 = g_q.z * g_q.z;

          /* Gradient descent algorithm corrective step */

          s0 = _2q2 * (2.0f * q1q1 + 2.0f * q2q2 - 1.0f - az) +
               (-_2q1) * (2.0f * q0q1 + 2.0f * q2q3 - ax) +
               (-_2q0) * (2.0f * q0q2 - 2.0f * q1q3 - ay);
          s1 = _2q3 * (2.0f * q1q1 + 2.0f * q2q2 - 1.0f - az) +
               _2q0 * (2.0f * q0q1 + 2.0f * q2q3 - ax) +
               _2q1 * (2.0f * q0q2 - 2.0f * q1q3 - ay);
          s2 = (-_2q0) * (2.0f * q1q1 + 2.0f * q2q2 - 1.0f - az) +
               _2q3 * (2.0f * q0q1 + 2.0f * q2q3 - ax) +
               _2q2 * (2.0f * q0q2 - 2.0f * q1q3 - ay);
          s3 = (-_2q1) * (2.0f * q1q1 + 2.0f * q2q2 - 1.0f - az) +
               (-_2q2) * (2.0f * q0q1 + 2.0f * q2q3 - ax);

          recipnorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
          s0 *= recipnorm;
          s1 *= recipnorm;
          s2 *= recipnorm;
          s3 *= recipnorm;

          /* Apply feedback step */

          qDot1 -= g_beta * s0;
          qDot2 -= g_beta * s1;
          qDot3 -= g_beta * s2;
          qDot4 -= g_beta * s3;
        }

      /* Integrate rate of change of quaternion */

      g_q.w += qDot1 * g_sample_period;
      g_q.x += qDot2 * g_sample_period;
      g_q.y += qDot3 * g_sample_period;
      g_q.z += qDot4 * g_sample_period;

      /* Normalize quaternion */

      recipnorm = inv_sqrt(g_q.w * g_q.w + g_q.x * g_q.x +
                           g_q.y * g_q.y + g_q.z * g_q.z);
      g_q.w *= recipnorm;
      g_q.x *= recipnorm;
      g_q.y *= recipnorm;
      g_q.z *= recipnorm;

      return;
    }

  /* Normalize accelerometer measurement */

  recipnorm = inv_sqrt(ax * ax + ay * ay + az * az);
  ax *= recipnorm;
  ay *= recipnorm;
  az *= recipnorm;

  /* Normalize magnetometer measurement */

  recipnorm = inv_sqrt(mx * mx + my * my + mz * mz);
  mx *= recipnorm;
  my *= recipnorm;
  mz *= recipnorm;

  /* Auxiliary variables to avoid repeated arithmetic */

  _2q0mx = 2.0f * g_q.w * mx;
  _2q0my = 2.0f * g_q.w * my;
  _2q0mz = 2.0f * g_q.w * mz;
  _2q1mx = 2.0f * g_q.x * mx;
  _2q0 = 2.0f * g_q.w;
  _2q1 = 2.0f * g_q.x;
  _2q2 = 2.0f * g_q.y;
  _2q3 = 2.0f * g_q.z;
  _2q0q2 = 2.0f * g_q.w * g_q.y;
  _2q2q3 = 2.0f * g_q.y * g_q.z;
  q0q0 = g_q.w * g_q.w;
  q0q1 = g_q.w * g_q.x;
  q0q2 = g_q.w * g_q.y;
  q0q3 = g_q.w * g_q.z;
  q1q1 = g_q.x * g_q.x;
  q1q2 = g_q.x * g_q.y;
  q1q3 = g_q.x * g_q.z;
  q2q2 = g_q.y * g_q.y;
  q2q3 = g_q.y * g_q.z;
  q3q3 = g_q.z * g_q.z;

  /* Reference direction of Earth's magnetic field */

  hx = mx * q0q0 - _2q0my * g_q.z + _2q0mz * g_q.y + mx * q1q1 +
       _2q1 * my * g_q.y + _2q1 * mz * g_q.z - mx * q2q2 - mx * q3q3;
  hy = _2q0mx * g_q.z + my * q0q0 - _2q0mz * g_q.x + _2q1mx * g_q.y -
       my * q1q1 + my * q2q2 + _2q2 * mz * g_q.z - my * q3q3;
  _2bx = sqrtf(hx * hx + hy * hy);
  _2bz = -_2q0mx * g_q.y + _2q0my * g_q.x + mz * q0q0 + _2q1mx * g_q.z -
         mz * q1q1 + _2q2 * my * g_q.z - mz * q2q2 + mz * q3q3;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  /* Gradient descent algorithm corrective step */

  s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) +
       _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
       _2bz * g_q.y * (_2bx * (0.5f - q2q2 - q3q3) +
                       _2bz * (q1q3 - q0q2) - mx) +
       (-_2bx * g_q.z + _2bz * g_q.x) *
       (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
       _2bx * g_q.y * (_2bx * (q0q2 + q1q3) +
                       _2bz * (0.5f - q1q1 - q2q2) - mz);

  s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) +
       _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
       4.0f * g_q.x * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
       _2bz * g_q.z * (_2bx * (0.5f - q2q2 - q3q3) +
                       _2bz * (q1q3 - q0q2) - mx) +
       (_2bx * g_q.y + _2bz * g_q.w) *
       (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
       (_2bx * g_q.z - _4bz * g_q.x) *
       (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

  s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) +
       _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
       4.0f * g_q.y * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
       (-_4bx * g_q.y - _2bz * g_q.w) *
       (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
       (_2bx * g_q.x + _2bz * g_q.z) *
       (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
       (_2bx * g_q.w - _4bz * g_q.y) *
       (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

  s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) +
       _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
       (-_4bx * g_q.z + _2bz * g_q.x) *
       (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
       (-_2bx * g_q.w + _2bz * g_q.y) *
       (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
       _2bx * g_q.x * (_2bx * (q0q2 + q1q3) +
                       _2bz * (0.5f - q1q1 - q2q2) - mz);

  recipnorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
  s0 *= recipnorm;
  s1 *= recipnorm;
  s2 *= recipnorm;
  s3 *= recipnorm;

  /* Rate of change of quaternion from gyroscope */

  qDot1 = 0.5f * (-g_q.x * gx - g_q.y * gy - g_q.z * gz) - g_beta * s0;
  qDot2 = 0.5f * (g_q.w * gx + g_q.y * gz - g_q.z * gy) - g_beta * s1;
  qDot3 = 0.5f * (g_q.w * gy - g_q.x * gz + g_q.z * gx) - g_beta * s2;
  qDot4 = 0.5f * (g_q.w * gz + g_q.x * gy - g_q.y * gx) - g_beta * s3;

  /* Integrate to yield quaternion */

  g_q.w += qDot1 * g_sample_period;
  g_q.x += qDot2 * g_sample_period;
  g_q.y += qDot3 * g_sample_period;
  g_q.z += qDot4 * g_sample_period;

  /* Normalize quaternion */

  recipnorm = inv_sqrt(g_q.w * g_q.w + g_q.x * g_q.x +
                       g_q.y * g_q.y + g_q.z * g_q.z);
  g_q.w *= recipnorm;
  g_q.x *= recipnorm;
  g_q.y *= recipnorm;
  g_q.z *= recipnorm;
}

/****************************************************************************
 * Name: fusion_get_quaternion
 ****************************************************************************/

void fusion_get_quaternion(quaternion_t *q)
{
  if (q)
    {
      *q = g_q;
    }
}

/****************************************************************************
 * Name: fusion_get_euler
 ****************************************************************************/

void fusion_get_euler(euler_angles_t *euler)
{
  if (euler)
    {
      /* Roll (x-axis rotation) */

      float sinr_cosp = 2.0f * (g_q.w * g_q.x + g_q.y * g_q.z);
      float cosr_cosp = 1.0f - 2.0f * (g_q.x * g_q.x + g_q.y * g_q.y);
      euler->roll = atan2f(sinr_cosp, cosr_cosp);

      /* Pitch (y-axis rotation) */

      float sinp = 2.0f * (g_q.w * g_q.y - g_q.z * g_q.x);
      if (fabsf(sinp) >= 1.0f)
        {
          euler->pitch = copysignf(M_PI / 2.0f, sinp);
        }
      else
        {
          euler->pitch = asinf(sinp);
        }

      /* Yaw (z-axis rotation) */

      float siny_cosp = 2.0f * (g_q.w * g_q.z + g_q.x * g_q.y);
      float cosy_cosp = 1.0f - 2.0f * (g_q.y * g_q.y + g_q.z * g_q.z);
      euler->yaw = atan2f(siny_cosp, cosy_cosp);
    }
}

/****************************************************************************
 * Name: fusion_get_result
 ****************************************************************************/

void fusion_get_result(fusion_result_t *result)
{
  if (result)
    {
      fusion_get_quaternion(&result->quaternion);
      fusion_get_euler(&result->euler);
      result->timestamp = clock_systime_ticks() * (1000000 / CLK_TCK);
    }
}
