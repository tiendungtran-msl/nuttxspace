/****************************************************************************
 * apps/examples/uav_states_v1/lib/attitude_estimator/attitude_estimator_q.hpp
 *
 * Quaternion-based Attitude Estimator - inspired by PX4 attitude_estimator_q
 * 
 * Key improvements over simple complementary filter:
 * - Adaptive accel weight based on acceleration magnitude
 * - Online gyro bias estimation
 * - Spin rate compensation
 * - Stable yaw (no drift when stationary)
 * 
 * Reference: PX4-Autopilot/src/modules/attitude_estimator_q
 ****************************************************************************/

#pragma once

#include <cmath>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

namespace attitude
{

//=============================================================================
// Vector3f - 3D vector
//=============================================================================

struct Vector3f
{
    float x, y, z;
    
    Vector3f() : x(0), y(0), z(0) {}
    Vector3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    float norm() const { return sqrtf(x*x + y*y + z*z); }
    float norm_squared() const { return x*x + y*y + z*z; }
    
    void zero() { x = y = z = 0.0f; }
    
    void normalize()
    {
        float n = norm();
        if (n > 1e-6f) {
            float inv = 1.0f / n;
            x *= inv; y *= inv; z *= inv;
        }
    }
    
    Vector3f normalized() const
    {
        Vector3f v = *this;
        v.normalize();
        return v;
    }
    
    Vector3f operator+(const Vector3f& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vector3f operator-(const Vector3f& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vector3f operator*(float s) const { return {x*s, y*s, z*s}; }
    Vector3f& operator+=(const Vector3f& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vector3f& operator-=(const Vector3f& o) { x-=o.x; y-=o.y; z-=o.z; return *this; }
    
    // Dot product
    float operator*(const Vector3f& o) const { return x*o.x + y*o.y + z*o.z; }
    
    // Cross product
    Vector3f operator%(const Vector3f& o) const 
    {
        return {
            y*o.z - z*o.y,
            z*o.x - x*o.z,
            x*o.y - y*o.x
        };
    }
};

//=============================================================================
// Quaternion
//=============================================================================

struct Quatf
{
    float w, x, y, z;
    
    Quatf() : w(1), x(0), y(0), z(0) {}
    Quatf(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
    
    void normalize()
    {
        float n = sqrtf(w*w + x*x + y*y + z*z);
        if (n > 1e-6f) {
            float inv = 1.0f / n;
            w *= inv; x *= inv; y *= inv; z *= inv;
        }
    }
    
    bool isAllFinite() const
    {
        return std::isfinite(w) && std::isfinite(x) && 
               std::isfinite(y) && std::isfinite(z);
    }
    
    // Quaternion multiplication
    Quatf operator*(const Quatf& q) const
    {
        return {
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        };
    }
    
    // Rotate vector by quaternion: v' = q * [0,v] * q^-1
    Vector3f rotateVector(const Vector3f& v) const
    {
        // Optimized quaternion-vector rotation
        float qwqw = w * w;
        float qwqx = w * x;
        float qwqy = w * y;
        float qwqz = w * z;
        float qxqx = x * x;
        float qxqy = x * y;
        float qxqz = x * z;
        float qyqy = y * y;
        float qyqz = y * z;
        float qzqz = z * z;
        
        return {
            v.x * (qwqw + qxqx - qyqy - qzqz) + 2.0f * (v.y * (qxqy - qwqz) + v.z * (qxqz + qwqy)),
            v.y * (qwqw - qxqx + qyqy - qzqz) + 2.0f * (v.x * (qxqy + qwqz) + v.z * (qyqz - qwqx)),
            v.z * (qwqw - qxqx - qyqy + qzqz) + 2.0f * (v.x * (qxqz - qwqy) + v.y * (qyqz + qwqx))
        };
    }
    
    // Rotate vector by inverse quaternion: v' = q^-1 * [0,v] * q
    Vector3f rotateVectorInverse(const Vector3f& v) const
    {
        // q^-1 for unit quaternion is conjugate
        Quatf qi(w, -x, -y, -z);
        return qi.rotateVector(v);
    }
    
    // Quaternion derivative from angular velocity
    Quatf derivative(const Vector3f& omega) const
    {
        return {
            0.5f * (-x*omega.x - y*omega.y - z*omega.z),
            0.5f * ( w*omega.x + y*omega.z - z*omega.y),
            0.5f * ( w*omega.y - x*omega.z + z*omega.x),
            0.5f * ( w*omega.z + x*omega.y - y*omega.x)
        };
    }
    
    // Get Euler angles (ZYX convention)
    void toEuler(float& roll, float& pitch, float& yaw) const
    {
        // Roll (x-axis)
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        roll = atan2f(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis)
        float sinp = 2.0f * (w * y - z * x);
        if (fabsf(sinp) >= 1.0f) {
            pitch = copysignf(M_PI_F / 2.0f, sinp);
        } else {
            pitch = asinf(sinp);
        }
        
        // Yaw (z-axis)
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        yaw = atan2f(siny_cosp, cosy_cosp);
    }
};

//=============================================================================
// Euler Angles Output
//=============================================================================

struct EulerAngles
{
    float roll, pitch, yaw;  // radians
    
    EulerAngles() : roll(0), pitch(0), yaw(0) {}
    EulerAngles(float r, float p, float y) : roll(r), pitch(p), yaw(y) {}
    
    void to_degrees(float& r_deg, float& p_deg, float& y_deg) const
    {
        constexpr float RAD_TO_DEG = 57.295779513f;
        r_deg = roll * RAD_TO_DEG;
        p_deg = pitch * RAD_TO_DEG;
        y_deg = yaw * RAD_TO_DEG;
    }
};

//=============================================================================
// AttitudeEstimatorQ - PX4-style quaternion estimator
//=============================================================================

class AttitudeEstimatorQ
{
public:
    AttitudeEstimatorQ();
    
    /**
     * @brief Update estimator with new IMU data
     * @param accel Accelerometer (m/s²), body frame
     * @param gyro Gyroscope (rad/s), body frame
     * @param dt Time step (seconds)
     * @return true if attitude is valid
     */
    bool update(const float accel[3], const float gyro[3], float dt);
    
    /**
     * @brief Get current attitude
     */
    EulerAngles get_euler() const;
    Quatf get_quaternion() const { return _q; }
    
    /**
     * @brief Get estimated gyro bias
     */
    Vector3f get_gyro_bias() const { return _gyro_bias; }
    
    /**
     * @brief Get body angular rates (bias-corrected)
     */
    Vector3f get_rates() const { return _rates; }
    
    /**
     * @brief Reset to initial state
     */
    void reset();
    
    /**
     * @brief Initialize attitude from accelerometer
     */
    bool init_from_accel(const float accel[3]);
    
    //-------------------------------------------------------------------------
    // Parameters (can be tuned)
    //-------------------------------------------------------------------------
    
    void set_w_acc(float w) { _w_acc = w; }       // Accel weight (0.1 - 0.3)
    void set_w_gyro_bias(float w) { _w_gyro_bias = w; }  // Bias estimation rate
    void set_bias_max(float b) { _bias_max = b; } // Max gyro bias (rad/s)

private:
    // State
    Quatf _q;                   ///< Attitude quaternion
    Vector3f _gyro_bias;        ///< Estimated gyro bias
    Vector3f _rates;            ///< Corrected angular rates
    
    // Status
    bool _initialized;
    
    // Parameters (PX4 defaults)
    float _w_acc;               ///< Accel correction weight (0.2)
    float _w_gyro_bias;         ///< Gyro bias learning rate (0.1)
    float _bias_max;            ///< Max gyro bias (rad/s)
    
    // Constants
    static constexpr float CONSTANTS_ONE_G = 9.80665f;
    static constexpr float DT_MIN = 0.00001f;
    static constexpr float DT_MAX = 0.02f;
    
    /**
     * @brief Wrap angle to [-π, π]
     */
    static float wrap_pi(float angle);
};

} // namespace attitude
