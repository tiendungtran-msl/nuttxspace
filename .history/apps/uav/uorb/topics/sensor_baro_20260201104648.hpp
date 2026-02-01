/****************************************************************************
 * apps/uav/uorb/topics/sensor_baro.hpp
 *
 * Topic: sensor_baro - Dữ liệu Barometer
 *
 * PUBLISHER:  sensors_app (baro driver)
 * SUBSCRIBER: estimator_app
 *
 * MÔ TẢ:
 * - Chứa áp suất khí quyển
 * - Độ cao được tính từ áp suất (không chính xác tuyệt đối)
 * - Dùng kết hợp với GPS cho altitude fusion
 *
 * TẦN SỐ: 50 Hz
 ****************************************************************************/

#pragma once

#include "../orb_defines.hpp"

/**
 * @brief Mẫu dữ liệu Barometer
 */
struct sensor_baro_s
{
    uint64_t timestamp_us;   ///< Timestamp [µs since boot]

    float pressure;          ///< Áp suất [Pa]
    float altitude;          ///< Độ cao tính từ pressure [m]
    float temperature;       ///< Nhiệt độ chip [°C]

    uint8_t instance;        ///< Sensor instance
    uint8_t _padding[7];     ///< Padding
};

ORB_DECLARE(sensor_baro);
