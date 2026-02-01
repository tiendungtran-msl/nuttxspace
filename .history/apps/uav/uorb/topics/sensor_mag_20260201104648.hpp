/****************************************************************************
 * apps/uav/uorb/topics/sensor_mag.hpp
 *
 * Topic: sensor_mag - Dữ liệu Magnetometer
 *
 * PUBLISHER:  sensors_app (mag driver)
 * SUBSCRIBER: estimator_app
 *
 * MÔ TẢ:
 * - Chứa từ trường đo được theo 3 trục
 * - Dùng để tính heading (hướng la bàn)
 * - Cần calibration hard/soft iron
 *
 * TẦN SỐ: 100 Hz
 ****************************************************************************/

#pragma once

#include "../orb_defines.hpp"

/**
 * @brief Mẫu dữ liệu Magnetometer
 *
 * @note Đơn vị: Gauss (1 Gauss = 100 µT)
 */
struct sensor_mag_s
{
    uint64_t timestamp_us;   ///< Timestamp [µs since boot]

    float field[3];          ///< Từ trường [Gauss] (X, Y, Z body frame)
    float temperature;       ///< Nhiệt độ chip [°C]

    uint8_t instance;        ///< Sensor instance
    uint8_t _padding[7];     ///< Padding
};

ORB_DECLARE(sensor_mag);
