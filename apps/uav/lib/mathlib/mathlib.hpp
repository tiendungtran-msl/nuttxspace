/****************************************************************************
 * apps/uav/lib/mathlib/mathlib.hpp
 *
 * Master include file cho mathlib
 *
 ****************************************************************************/

#ifndef UAV_LIB_MATHLIB_MATHLIB_HPP
#define UAV_LIB_MATHLIB_MATHLIB_HPP

#include "vector3.hpp"
#include "matrix3.hpp"
#include "quaternion.hpp"
#include "filters.hpp"

namespace mathlib {

/****************************************************************************
 * Constants
 ****************************************************************************/

constexpr float DEG_TO_RAD = 0.017453292519943295f;  // PI / 180
constexpr float RAD_TO_DEG = 57.295779513082323f;    // 180 / PI

constexpr float GRAVITY_MSS = 9.80665f;              // Standard gravity

/****************************************************************************
 * Utility Functions
 ****************************************************************************/

/**
 * Constrain value to range
 */
template<typename T>
inline T constrain(T val, T min_val, T max_val) {
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

/**
 * Wrap angle to [-PI, PI]
 */
inline float wrap_pi(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/**
 * Wrap angle to [0, 2*PI]
 */
inline float wrap_2pi(float angle) {
    while (angle >= 2.0f * M_PI) angle -= 2.0f * M_PI;
    while (angle < 0.0f) angle += 2.0f * M_PI;
    return angle;
}

/**
 * Linear interpolation
 */
inline float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

/**
 * Safe sqrt (returns 0 for negative input)
 */
inline float safe_sqrt(float x) {
    return (x <= 0.0f) ? 0.0f : sqrtf(x);
}

/**
 * Inverse sqrt approximation (fast)
 */
inline float inv_sqrt(float x) {
    return 1.0f / sqrtf(x);
}

/**
 * Sign function
 */
inline float sign(float x) {
    return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f);
}

} // namespace mathlib

#endif // UAV_LIB_MATHLIB_MATHLIB_HPP
