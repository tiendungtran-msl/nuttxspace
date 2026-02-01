/****************************************************************************
 * apps/examples/uav_states_v1/lib/attitude_estimator/attitude_estimator_q.cpp
 *
 * Implementation based on PX4's attitude_estimator_q
 * 
 * Key algorithm:
 * 1. Integrate gyro to predict attitude (fast, but drifts)
 * 2. Use accel to measure gravity direction → correct roll/pitch
 * 3. Estimate gyro bias online when stationary
 * 4. Yaw: only gyro integration (no magnetometer), accept drift
 * 
 * Anti-drift measures:
 * - Accel fusion only when |accel| ≈ 1g (rejects dynamics)
 * - Bias estimation only at low spin rates
 * - Adaptive gain based on spin rate
 ****************************************************************************/

#include "attitude_estimator_q.hpp"
#include <cstring>

namespace attitude
{

AttitudeEstimatorQ::AttitudeEstimatorQ()
    : _initialized(false)
    , _w_acc(0.2f)
    , _w_gyro_bias(0.1f)
    , _bias_max(0.05f)  // ~3 deg/s max bias
{
    reset();
}

void AttitudeEstimatorQ::reset()
{
    _q = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
    _gyro_bias.zero();
    _rates.zero();
    _initialized = false;
}

bool AttitudeEstimatorQ::init_from_accel(const float accel[3])
{
    Vector3f a(accel[0], accel[1], accel[2]);
    float norm = a.norm();
    
    // Check valid acceleration
    if (norm < 0.5f * CONSTANTS_ONE_G || norm > 1.5f * CONSTANTS_ONE_G) {
        return false;
    }
    
    // Normalize: k = -accel (down in NED, but accel measures up)
    Vector3f k = a * (-1.0f / norm);
    
    // Build rotation matrix from gravity
    // Assume yaw = 0 (north)
    // k is the Down axis in body frame
    
    // i = North direction (arbitrary, perpendicular to k)
    // Choose i in XZ plane of body for zero yaw
    Vector3f i;
    if (fabsf(k.z) < 0.9f) {
        // k not pointing up/down, use standard method
        i = Vector3f(1.0f, 0.0f, 0.0f);
        i = i - k * (i * k);  // Remove component along k
        i.normalize();
    } else {
        // Near gimbal lock, use different approach
        i = Vector3f(k.z > 0 ? 1.0f : -1.0f, 0.0f, 0.0f);
    }
    
    // j = k × i
    Vector3f j = k % i;
    j.normalize();
    
    // Recalculate i to ensure orthogonality
    i = j % k;
    i.normalize();
    
    // Build DCM and convert to quaternion
    // DCM rows are i, j, k (body to NED)
    // Using simplified conversion
    
    float trace = i.x + j.y + k.z;
    
    if (trace > 0.0f) {
        float s = 0.5f / sqrtf(trace + 1.0f);
        _q.w = 0.25f / s;
        _q.x = (j.z - k.y) * s;
        _q.y = (k.x - i.z) * s;
        _q.z = (i.y - j.x) * s;
    } else if (i.x > j.y && i.x > k.z) {
        float s = 2.0f * sqrtf(1.0f + i.x - j.y - k.z);
        _q.w = (j.z - k.y) / s;
        _q.x = 0.25f * s;
        _q.y = (j.x + i.y) / s;
        _q.z = (k.x + i.z) / s;
    } else if (j.y > k.z) {
        float s = 2.0f * sqrtf(1.0f + j.y - i.x - k.z);
        _q.w = (k.x - i.z) / s;
        _q.x = (j.x + i.y) / s;
        _q.y = 0.25f * s;
        _q.z = (k.y + j.z) / s;
    } else {
        float s = 2.0f * sqrtf(1.0f + k.z - i.x - j.y);
        _q.w = (i.y - j.x) / s;
        _q.x = (k.x + i.z) / s;
        _q.y = (k.y + j.z) / s;
        _q.z = 0.25f * s;
    }
    
    _q.normalize();
    _initialized = true;
    return true;
}

bool AttitudeEstimatorQ::update(const float accel[3], const float gyro[3], float dt)
{
    // Validate dt
    if (dt < DT_MIN || dt > DT_MAX) {
        return false;
    }
    
    Vector3f accel_vec(accel[0], accel[1], accel[2]);
    Vector3f gyro_vec(gyro[0], gyro[1], gyro[2]);
    
    // Initialize if needed
    if (!_initialized) {
        if (!init_from_accel(accel)) {
            return false;
        }
    }
    
    Quatf q_last = _q;
    
    // Correction vector (error to apply)
    Vector3f corr;
    corr.zero();
    
    // Spin rate (for adaptive gains)
    float spinRate = gyro_vec.norm();
    
    //-------------------------------------------------------------------------
    // Accelerometer correction (roll/pitch only)
    //-------------------------------------------------------------------------
    
    float accel_norm_sq = accel_vec.norm_squared();
    float upper_limit = CONSTANTS_ONE_G * 1.1f;
    float lower_limit = CONSTANTS_ONE_G * 0.9f;
    
    // Only fuse accel when magnitude is close to 1g
    // This rejects accelerations from motion
    if (accel_norm_sq > (lower_limit * lower_limit) &&
        accel_norm_sq < (upper_limit * upper_limit)) {
        
        // Estimated gravity direction in body frame from quaternion
        // k = q.rotateVectorInverse([0, 0, 1]) = down in body
        // Optimized version:
        Vector3f k(
            2.0f * (_q.x * _q.z - _q.w * _q.y),
            2.0f * (_q.w * _q.x + _q.y * _q.z),
            (_q.w * _q.w - _q.x * _q.x - _q.y * _q.y + _q.z * _q.z)
        );
        
        // Measured gravity direction (normalized, inverted because accel = -gravity)
        Vector3f accel_norm = accel_vec.normalized() * (-1.0f);
        
        // Error: cross product of measured and estimated gravity
        // This gives rotation axis to align estimated with measured
        corr = k % accel_norm;
        
        // Scale by accel weight
        corr = corr * _w_acc;
    }
    
    //-------------------------------------------------------------------------
    // Gyro bias estimation
    //-------------------------------------------------------------------------
    
    // Only update bias when spinning slowly (stationary)
    if (spinRate < 0.175f) {  // ~10 deg/s
        _gyro_bias += corr * (_w_gyro_bias * dt);
        
        // Limit bias magnitude
        for (int i = 0; i < 3; i++) {
            float* b = (i == 0) ? &_gyro_bias.x : (i == 1) ? &_gyro_bias.y : &_gyro_bias.z;
            if (*b > _bias_max) *b = _bias_max;
            if (*b < -_bias_max) *b = -_bias_max;
        }
    }
    
    //-------------------------------------------------------------------------
    // Compute corrected rates
    //-------------------------------------------------------------------------
    
    _rates = gyro_vec - _gyro_bias;
    
    // Add correction to rates
    Vector3f omega = _rates + corr;
    
    //-------------------------------------------------------------------------
    // Integrate quaternion
    //-------------------------------------------------------------------------
    
    Quatf q_dot = _q.derivative(omega);
    
    _q.w += q_dot.w * dt;
    _q.x += q_dot.x * dt;
    _q.y += q_dot.y * dt;
    _q.z += q_dot.z * dt;
    
    _q.normalize();
    
    // Sanity check
    if (!_q.isAllFinite()) {
        _q = q_last;
        _rates.zero();
        _gyro_bias.zero();
        return false;
    }
    
    return true;
}

EulerAngles AttitudeEstimatorQ::get_euler() const
{
    EulerAngles e;
    _q.toEuler(e.roll, e.pitch, e.yaw);
    return e;
}

float AttitudeEstimatorQ::wrap_pi(float angle)
{
    while (angle > M_PI_F) angle -= 2.0f * M_PI_F;
    while (angle < -M_PI_F) angle += 2.0f * M_PI_F;
    return angle;
}

} // namespace attitude
