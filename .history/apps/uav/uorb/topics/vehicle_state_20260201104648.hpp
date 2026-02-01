/****************************************************************************
 * apps/uav/uorb/topics/vehicle_state.hpp
 *
 * Topic: vehicle_state - Trạng thái tổng thể của UAV
 *
 * PUBLISHER:  state_app
 * SUBSCRIBER: Tất cả apps cần biết mode hiện tại
 *
 * MÔ TẢ:
 * - Mode hiện tại (INIT, PREFLIGHT, ARMED, ...)
 * - Arming state
 * - Health status
 *
 * TẦN SỐ: 10 Hz
 ****************************************************************************/

#pragma once

#include "../orb_defines.hpp"

/**
 * @brief Vehicle mode enum
 */
enum class VehicleMode : uint8_t
{
    INIT = 0,        ///< Đang khởi tạo
    PREFLIGHT,       ///< Sẵn sàng, chờ arm
    ARMED,           ///< Đã arm, sẵn sàng bay
    FLIGHT,          ///< Đang bay
    LANDING,         ///< Đang hạ cánh
    FAILSAFE,        ///< Lỗi, chế độ an toàn
    DISARMED         ///< Đã disarm
};

/**
 * @brief Arming state
 */
enum class ArmingState : uint8_t
{
    DISARMED = 0,
    ARMED,
    ARMED_ERROR      ///< Armed nhưng có lỗi
};

/**
 * @brief Trạng thái vehicle
 */
struct vehicle_state_s
{
    uint64_t timestamp_us;   ///< Timestamp [µs since boot]

    // Mode
    uint8_t mode;            ///< VehicleMode
    uint8_t arming_state;    ///< ArmingState

    // Health flags
    bool imu_healthy;        ///< Ít nhất 1 IMU hoạt động
    bool mag_healthy;        ///< Magnetometer hoạt động
    bool baro_healthy;       ///< Barometer hoạt động
    bool gps_healthy;        ///< GPS có fix

    bool attitude_valid;     ///< Attitude estimate valid
    bool position_valid;     ///< Position estimate valid

    // Counters
    uint32_t flight_time_s;  ///< Thời gian bay [s]

    uint8_t _padding[4];
};

ORB_DECLARE(vehicle_state);
