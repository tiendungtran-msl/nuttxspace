/****************************************************************************
 * apps/examples/imu_system/algorithms/orientation.c
 *
 * Orientation Calculation Implementation
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <math.h>

#include "orientation.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static float g_heading_rad = 0.0f;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: orientation_init
 ****************************************************************************/

void orientation_init(void)
{
  g_heading_rad = 0.0f;
}

/****************************************************************************
 * Name: orientation_calculate
 ****************************************************************************/

float orientation_calculate(float roll, float pitch,
                            float mx, float my, float mz)
{
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  float mag_x;
  float mag_y;

  /* Apply tilt compensation to magnetometer readings */

  cos_roll = cosf(roll);
  sin_roll = sinf(roll);
  cos_pitch = cosf(pitch);
  sin_pitch = sinf(pitch);

  /* Tilt compensated magnetic field X component */

  mag_x = mx * cos_pitch + my * sin_roll * sin_pitch +
          mz * cos_roll * sin_pitch;

  /* Tilt compensated magnetic field Y component */

  mag_y = my * cos_roll - mz * sin_roll;

  /* Calculate heading */

  g_heading_rad = atan2f(-mag_y, mag_x);

  return g_heading_rad;
}

/****************************************************************************
 * Name: orientation_get_heading
 ****************************************************************************/

float orientation_get_heading(void)
{
  float heading_deg = g_heading_rad * RAD_TO_DEG;

  /* Normalize to 0-360 degrees */

  if (heading_deg < 0.0f)
    {
      heading_deg += 360.0f;
    }

  return heading_deg;
}
