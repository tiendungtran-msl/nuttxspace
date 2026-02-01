/****************************************************************************
 * apps/uav/uorb/topics/sensor_imu.hpp
 *
 * Topic: sensor_imu - Dữ liệu IMU thô
 *
 * PUBLISHER:  sensors_app (imu driver)
 * SUBSCRIBER: estimator_app
 *
 * MÔ TẢ:
 * - Chứa dữ liệu accelerometer và gyroscope
 * - Đã bù bias calibration
 * - Tọa độ body frame (X forward, Y right, Z down)
 *
 * TẦN SỐ: 1000 Hz (mỗi 1ms)
 ****************************************************************************/

#pragma once

#include "../orb_defines.hpp"

/**
 * @brief Mẫu dữ liệu IMU
 *
 * @note Tất cả dữ liệu theo hệ SI:
 *       - Gia tốc: m/s²
 *       - Vận tốc góc: rad/s
 *       - Nhiệt độ: °C
 */
struct sensor_imu_s
{
    uint64_t timestamp_us;   ///< Timestamp [µs since boot]

    float accel[3];          ///< Gia tốc [m/s²] (X, Y, Z body frame)
    float gyro[3];           ///< Vận tốc góc [rad/s] (X, Y, Z body frame)
    float temperature;       ///< Nhiệt độ chip [°C]

    uint8_t instance;        ///< Sensor instance (0-3 cho multi-IMU)
    uint8_t _padding[3];     ///< Padding để align 4 bytes
};

// Đăng ký topic
ORB_DECLARE(sensor_imu);
