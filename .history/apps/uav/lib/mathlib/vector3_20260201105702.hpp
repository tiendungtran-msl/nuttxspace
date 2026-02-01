/****************************************************************************
 * apps/uav/lib/mathlib/vector3.hpp
 *
 * 3D Vector Class
 *
 * MỤC ĐÍCH:
 * - Biểu diễn vectors 3D (velocity, position, angular rate)
 * - Các phép tính: dot, cross, normalize
 *
 ****************************************************************************/

#ifndef UAV_LIB_MATHLIB_VECTOR3_HPP
#define UAV_LIB_MATHLIB_VECTOR3_HPP

#include <cmath>

namespace mathlib {

/****************************************************************************
 * Vector3 Class
 ****************************************************************************/

class Vector3 {
public:
    float x, y, z;

    //=========================================================================
    // Constructors
    //=========================================================================

    Vector3() : x(0), y(0), z(0) {}

    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    explicit Vector3(const float* arr) : x(arr[0]), y(arr[1]), z(arr[2]) {}

    //=========================================================================
    // Operations
    //=========================================================================

    /**
     * Norm (magnitude)
     */
    float norm() const {
        return sqrtf(x*x + y*y + z*z);
    }

    /**
     * Squared norm (avoid sqrt)
     */
    float norm_squared() const {
        return x*x + y*y + z*z;
    }

    /**
     * Normalize in-place
     */
    void normalize() {
        float n = norm();
        if (n > 0) {
            float inv_n = 1.0f / n;
            x *= inv_n;
            y *= inv_n;
            z *= inv_n;
        }
    }

    /**
     * Return normalized copy
     */
    Vector3 normalized() const {
        Vector3 v = *this;
        v.normalize();
        return v;
    }

    /**
     * Dot product
     */
    float dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    /**
     * Cross product: this × other
     */
    Vector3 cross(const Vector3& other) const {
        return Vector3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    //=========================================================================
    // Operators
    //=========================================================================

    Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }

    Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }

    Vector3 operator*(float s) const {
        return Vector3(x * s, y * s, z * s);
    }

    Vector3 operator/(float s) const {
        float inv_s = 1.0f / s;
        return Vector3(x * inv_s, y * inv_s, z * inv_s);
    }

    Vector3& operator+=(const Vector3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vector3& operator*=(float s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    Vector3 operator-() const {
        return Vector3(-x, -y, -z);
    }

    //=========================================================================
    // Array access
    //=========================================================================

    float& operator[](int i) {
        return (&x)[i];
    }

    float operator[](int i) const {
        return (&x)[i];
    }

    void to_array(float arr[3]) const {
        arr[0] = x;
        arr[1] = y;
        arr[2] = z;
    }

    //=========================================================================
    // Static utility functions
    //=========================================================================

    /**
     * Zero vector
     */
    static Vector3 zero() {
        return Vector3(0, 0, 0);
    }

    /**
     * Unit vectors
     */
    static Vector3 unit_x() { return Vector3(1, 0, 0); }
    static Vector3 unit_y() { return Vector3(0, 1, 0); }
    static Vector3 unit_z() { return Vector3(0, 0, 1); }
};

// Scalar * Vector
inline Vector3 operator*(float s, const Vector3& v) {
    return v * s;
}

} // namespace mathlib

#endif // UAV_LIB_MATHLIB_VECTOR3_HPP
