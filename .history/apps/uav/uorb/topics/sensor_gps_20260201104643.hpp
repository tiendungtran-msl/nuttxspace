/****************************************************************************
 * apps/uav/uorb/topics/sensor_gps.hpp
 *
 * Topic: sensor_gps - Dữ liệu GPS
 *
 * PUBLISHER:  sensors_app (gps driver)
 * SUBSCRIBER: estimator_app
 *
 * MÔ TẢ:
 * - Vị trí địa lý (lat/lon/alt)
 * - Vận tốc NED
 * - Độ chính xác (accuracy)
 * - Thông tin fix và vệ tinh
 *
 * TẦN SỐ: 5-10 Hz (tùy GPS module)
 ****************************************************************************/

#pragma once

#include "../orb_defines.hpp"

/**
 * @brief Mẫu dữ liệu GPS
 */
struct sensor_gps_s
{
    uint64_t timestamp_us;   ///< Timestamp [µs since boot]

    // Vị trí
    double lat;              ///< Latitude [deg] (WGS84)
    double lon;              ///< Longitude [deg] (WGS84)
    float alt;               ///< Altitude MSL [m]

    // Vận tốc NED
    float vel_n;             ///< Velocity North [m/s]
    float vel_e;             ///< Velocity East [m/s]
    float vel_d;             ///< Velocity Down [m/s]

    // Độ chính xác
    float hacc;              ///< Horizontal accuracy [m]
    float vacc;              ///< Vertical accuracy [m]
    float sacc;              ///< Speed accuracy [m/s]

    // Trạng thái
    uint8_t fix_type;        ///< Fix type: 0=no, 2=2D, 3=3D, 4=RTK
    uint8_t nsats;           ///< Số vệ tinh
    uint8_t _padding[6];
};

ORB_DECLARE(sensor_gps);
