/****************************************************************************
 * apps/uav/uorb/topics/vehicle_local_position.hpp
 *
 * Topic: vehicle_local_position - Vị trí cục bộ NED
 *
 * PUBLISHER:  estimator_app (EKF2)
 * SUBSCRIBER: state_app, navigator, controller
 *
 * MÔ TẢ:
 * - Vị trí và vận tốc trong hệ NED
 * - Gốc tọa độ = điểm khởi động hoặc home
 * - Fused từ GPS + IMU + Baro
 *
 * TẦN SỐ: 50-100 Hz
 ****************************************************************************/

#pragma once

#include "../orb_defines.hpp"

/**
 * @brief Vị trí cục bộ NED
 *
 * FRAME: North-East-Down (NED)
 * - X: North (hướng Bắc)
 * - Y: East (hướng Đông)
 * - Z: Down (hướng xuống, dương = dưới gốc)
 */
struct vehicle_local_position_s
{
    uint64_t timestamp_us;   ///< Timestamp [µs since boot]

    // Vị trí NED [m]
    float x;                 ///< North position
    float y;                 ///< East position
    float z;                 ///< Down position (dương = dưới mặt đất)

    // Vận tốc NED [m/s]
    float vx;                ///< North velocity
    float vy;                ///< East velocity
    float vz;                ///< Down velocity

    // Gia tốc NED [m/s²] (cho smoothing/prediction)
    float ax;                ///< North acceleration
    float ay;                ///< East acceleration
    float az;                ///< Down acceleration

    // Origin (GPS reference)
    double ref_lat;          ///< Reference latitude [deg]
    double ref_lon;          ///< Reference longitude [deg]
    float ref_alt;           ///< Reference altitude MSL [m]

    // Validity flags
    bool xy_valid;           ///< xy position valid
    bool z_valid;            ///< z position valid
    bool v_xy_valid;         ///< xy velocity valid
    bool v_z_valid;          ///< z velocity valid

    uint8_t _padding[4];
};

ORB_DECLARE(vehicle_local_position);
