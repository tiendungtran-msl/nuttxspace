/****************************************************************************
 * apps/uav/lib/calibration/sensor_calibration.hpp
 *
 * Sensor calibration library - lấy cảm hứng từ PX4 sensor_calibration
 * Cung cấp bias, scale, và rotation matrix correction cho IMU sensors
 ****************************************************************************/

#pragma once

#include <cstdint>
#include <cmath>
#include <cstring>

namespace calibration
{

/**
 * @brief 3D vector structure cho sensor data
 */
struct Vector3f
{
    float x;
    float y;
    float z;

    Vector3f() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    float norm() const
    {
        return sqrtf(x * x + y * y + z * z);
    }

    void zero()
    {
        x = y = z = 0.0f;
    }

    bool is_finite() const
    {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }

    Vector3f operator-(const Vector3f &other) const
    {
        return Vector3f(x - other.x, y - other.y, z - other.z);
    }

    Vector3f operator+(const Vector3f &other) const
    {
        return Vector3f(x + other.x, y + other.y, z + other.z);
    }

    Vector3f operator*(float scalar) const
    {
        return Vector3f(x * scalar, y * scalar, z * scalar);
    }

    /* Element-wise multiplication */
    Vector3f emult(const Vector3f &other) const
    {
        return Vector3f(x * other.x, y * other.y, z * other.z);
    }

    /* Element-wise division */
    Vector3f edivide(const Vector3f &other) const
    {
        return Vector3f(x / other.x, y / other.y, z / other.z);
    }
};

/**
 * @brief 3x3 Direction Cosine Matrix cho rotation
 */
class Dcmf
{
public:
    float data[3][3];

    Dcmf()
    {
        /* Identity matrix */
        memset(data, 0, sizeof(data));
        data[0][0] = data[1][1] = data[2][2] = 1.0f;
    }

    /* Matrix-vector multiplication */
    Vector3f operator*(const Vector3f &v) const
    {
        return Vector3f(
            data[0][0] * v.x + data[0][1] * v.y + data[0][2] * v.z,
            data[1][0] * v.x + data[1][1] * v.y + data[1][2] * v.z,
            data[2][0] * v.x + data[2][1] * v.y + data[2][2] * v.z
        );
    }

    /* Transpose */
    Dcmf T() const
    {
        Dcmf result;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result.data[i][j] = data[j][i];
            }
        }
        return result;
    }

    /* Identity matrix */
    static Dcmf identity()
    {
        return Dcmf();
    }
};

/**
 * @brief Accelerometer calibration parameters và correction
 *
 * Model: accel_corrected = rotation * ((accel_raw - offset).emult(scale))
 *
 * - offset: bias in m/s² (sensor frame)
 * - scale: per-axis scale factors (dimensionless)
 * - rotation: sensor→body frame transformation (DCM)
 */
class Accelerometer
{
public:
    Accelerometer();

    /**
     * @brief Set accelerometer offset (bias)
     * @param offset_m_s2 Bias vector in m/s² (sensor frame)
     * @return true nếu updated
     */
    bool set_offset(const Vector3f &offset_m_s2);

    /**
     * @brief Set accelerometer scale factors
     * @param scale Per-axis scale factors (dimensionless)
     * @return true nếu updated
     */
    bool set_scale(const Vector3f &scale);

    /**
     * @brief Set rotation matrix (sensor→body frame)
     * @param rotation DCM for sensor-to-body transformation
     */
    void set_rotation(const Dcmf &rotation);

    /**
     * @brief Lấy offset hiện tại
     */
    const Vector3f& get_offset() const { return _offset; }

    /**
     * @brief Lấy scale hiện tại
     */
    const Vector3f& get_scale() const { return _scale; }

    /**
     * @brief Lấy rotation hiện tại
     */
    const Dcmf& get_rotation() const { return _rotation; }

    /**
     * @brief Apply calibration cho raw accelerometer data
     * @param raw_data Raw acceleration in m/s² (sensor frame)
     * @return Corrected acceleration in m/s² (body frame)
     */
    Vector3f correct(const Vector3f &raw_data) const;

    /**
     * @brief Reset về default (identity) calibration
     */
    void reset();

    /**
     * @brief Kiểm tra đã calibrate chưa
     */
    bool is_calibrated() const { return _calibration_count > 0; }

    /**
     * @brief Số lần calibration được update
     */
    uint8_t calibration_count() const { return _calibration_count; }

private:
    Vector3f _offset;           ///< Bias offset in m/s² (sensor frame)
    Vector3f _scale;            ///< Per-axis scale factors
    Dcmf _rotation;             ///< Sensor→body frame DCM
    uint8_t _calibration_count; ///< Number of calibration updates
};

/**
 * @brief Gyroscope calibration parameters và correction
 *
 * Model: gyro_corrected = rotation * (gyro_raw - offset)
 *
 * - offset: bias in rad/s (sensor frame)
 * - rotation: sensor→body frame transformation (DCM)
 */
class Gyroscope
{
public:
    Gyroscope();

    /**
     * @brief Set gyroscope offset (bias)
     * @param offset_rad_s Bias vector in rad/s (sensor frame)
     * @return true nếu updated
     */
    bool set_offset(const Vector3f &offset_rad_s);

    /**
     * @brief Set rotation matrix (sensor→body frame)
     * @param rotation DCM for sensor-to-body transformation
     */
    void set_rotation(const Dcmf &rotation);

    /**
     * @brief Lấy offset hiện tại
     */
    const Vector3f& get_offset() const { return _offset; }

    /**
     * @brief Lấy rotation hiện tại
     */
    const Dcmf& get_rotation() const { return _rotation; }

    /**
     * @brief Apply calibration cho raw gyroscope data
     * @param raw_data Raw angular velocity in rad/s (sensor frame)
     * @return Corrected angular velocity in rad/s (body frame)
     */
    Vector3f correct(const Vector3f &raw_data) const;

    /**
     * @brief Reset về default (identity) calibration
     */
    void reset();

    /**
     * @brief Kiểm tra đã calibrate chưa
     */
    bool is_calibrated() const { return _calibration_count > 0; }

    /**
     * @brief Số lần calibration được update
     */
    uint8_t calibration_count() const { return _calibration_count; }

private:
    Vector3f _offset;           ///< Bias offset in rad/s (sensor frame)
    Dcmf _rotation;             ///< Sensor→body frame DCM
    uint8_t _calibration_count; ///< Number of calibration updates
};

} // namespace calibration
