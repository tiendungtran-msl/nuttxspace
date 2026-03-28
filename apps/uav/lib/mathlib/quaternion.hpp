/****************************************************************************
 * apps/uav/lib/mathlib/quaternion.hpp
 *
 * Quaternion Class cho attitude representation
 *
 * MỤC ĐÍCH:
 * - Biểu diễn tư thế (attitude) bằng quaternion
 * - Các phép tính: nhân, inverse, normalize
 * - Chuyển đổi qua lại với Euler angles, rotation matrix
 *
 * CONVENTION:
 * - Hamilton convention (q = w + xi + yj + zk)
 * - q[0] = w (scalar), q[1-3] = xyz (vector)
 * - Right-handed coordinate system
 * - ZYX Euler angle sequence (yaw, pitch, roll)
 *
 ****************************************************************************/

#ifndef UAV_LIB_MATHLIB_QUATERNION_HPP
#define UAV_LIB_MATHLIB_QUATERNION_HPP

#include <cmath>
#include <cstring>

namespace mathlib {

/****************************************************************************
 * Quaternion Class
 ****************************************************************************/

class Quaternion {
public:
    float w, x, y, z;  // Hamilton: w + xi + yj + zk

    //=========================================================================
    // Constructors
    //=========================================================================

    // Default: identity quaternion
    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}

    // From components
    Quaternion(float w_, float x_, float y_, float z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    // From array [w, x, y, z]
    explicit Quaternion(const float* arr)
        : w(arr[0]), x(arr[1]), y(arr[2]), z(arr[3]) {}

    //=========================================================================
    // Factory Methods
    //=========================================================================

    /**
     * Tạo quaternion từ Euler angles (ZYX convention)
     *
     * @param roll  Góc roll (rad) - rotation quanh trục X
     * @param pitch Góc pitch (rad) - rotation quanh trục Y
     * @param yaw   Góc yaw (rad) - rotation quanh trục Z
     */
    static Quaternion from_euler(float roll, float pitch, float yaw) {
        float cr = cosf(roll * 0.5f);
        float sr = sinf(roll * 0.5f);
        float cp = cosf(pitch * 0.5f);
        float sp = sinf(pitch * 0.5f);
        float cy = cosf(yaw * 0.5f);
        float sy = sinf(yaw * 0.5f);

        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        return q;
    }

    /**
     * Tạo quaternion từ axis-angle representation
     *
     * @param axis  Rotation axis (normalized)
     * @param angle Rotation angle (rad)
     */
    static Quaternion from_axis_angle(const float axis[3], float angle) {
        float half_angle = angle * 0.5f;
        float s = sinf(half_angle);

        Quaternion q;
        q.w = cosf(half_angle);
        q.x = axis[0] * s;
        q.y = axis[1] * s;
        q.z = axis[2] * s;
        return q;
    }

    //=========================================================================
    // Conversion Methods
    //=========================================================================

    /**
     * Chuyển sang Euler angles (ZYX convention)
     */
    void to_euler(float* roll, float* pitch, float* yaw) const {
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        *roll = atan2f(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        float sinp = 2.0f * (w * y - z * x);
        if (fabsf(sinp) >= 1.0f) {
            *pitch = copysignf(1.5707963267948966f, sinp);  // Gimbal lock = PI/2
        } else {
            *pitch = asinf(sinp);
        }

        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        *yaw = atan2f(siny_cosp, cosy_cosp);
    }

    /**
     * Chuyển sang rotation matrix (DCM)
     * Output: 3x3 matrix, row-major
     */
    void to_dcm(float R[9]) const {
        float xx = x * x;
        float yy = y * y;
        float zz = z * z;
        float xy = x * y;
        float xz = x * z;
        float yz = y * z;
        float wx = w * x;
        float wy = w * y;
        float wz = w * z;

        R[0] = 1.0f - 2.0f * (yy + zz);
        R[1] = 2.0f * (xy - wz);
        R[2] = 2.0f * (xz + wy);

        R[3] = 2.0f * (xy + wz);
        R[4] = 1.0f - 2.0f * (xx + zz);
        R[5] = 2.0f * (yz - wx);

        R[6] = 2.0f * (xz - wy);
        R[7] = 2.0f * (yz + wx);
        R[8] = 1.0f - 2.0f * (xx + yy);
    }

    /**
     * Copy to array [w, x, y, z]
     */
    void to_array(float arr[4]) const {
        arr[0] = w;
        arr[1] = x;
        arr[2] = y;
        arr[3] = z;
    }

    //=========================================================================
    // Operations
    //=========================================================================

    /**
     * Norm (magnitude)
     */
    float norm() const {
        return sqrtf(w*w + x*x + y*y + z*z);
    }

    /**
     * Normalize in-place
     */
    void normalize() {
        float n = norm();
        if (n > 0.0f) {
            float inv_n = 1.0f / n;
            w *= inv_n;
            x *= inv_n;
            y *= inv_n;
            z *= inv_n;
        }
    }

    /**
     * Return normalized copy
     */
    Quaternion normalized() const {
        Quaternion q = *this;
        q.normalize();
        return q;
    }

    /**
     * Conjugate (inverse for unit quaternion)
     */
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    /**
     * Inverse
     */
    Quaternion inverse() const {
        float n_sq = w*w + x*x + y*y + z*z;
        if (n_sq > 0.0f) {
            float inv_n_sq = 1.0f / n_sq;
            return Quaternion(w * inv_n_sq, -x * inv_n_sq,
                              -y * inv_n_sq, -z * inv_n_sq);
        }
        return Quaternion();  // Identity
    }

    /**
     * Quaternion multiplication: this * other
     * Kết quả: rotation other rồi rotation this
     */
    Quaternion operator*(const Quaternion& other) const {
        return Quaternion(
            w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w
        );
    }

    /**
     * Rotate vector by this quaternion
     * v' = q * v * q^-1
     */
    void rotate_vector(const float v_in[3], float v_out[3]) const {
        // Optimized version using q * v * q^-1
        float qv_w = -x * v_in[0] - y * v_in[1] - z * v_in[2];
        float qv_x =  w * v_in[0] + y * v_in[2] - z * v_in[1];
        float qv_y =  w * v_in[1] - x * v_in[2] + z * v_in[0];
        float qv_z =  w * v_in[2] + x * v_in[1] - y * v_in[0];

        v_out[0] = -qv_w * x + qv_x * w - qv_y * z + qv_z * y;
        v_out[1] = -qv_w * y + qv_x * z + qv_y * w - qv_z * x;
        v_out[2] = -qv_w * z - qv_x * y + qv_y * x + qv_z * w;
    }

    /**
     * Integrate angular velocity
     * q_new = q * dq
     * dq = [1, omega * dt / 2]
     */
    void integrate(const float omega[3], float dt) {
        float half_dt = 0.5f * dt;

        Quaternion dq;
        dq.w = 1.0f;
        dq.x = omega[0] * half_dt;
        dq.y = omega[1] * half_dt;
        dq.z = omega[2] * half_dt;

        *this = *this * dq;
        normalize();
    }
};

} // namespace mathlib

#endif // UAV_LIB_MATHLIB_QUATERNION_HPP
