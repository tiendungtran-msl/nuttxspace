/****************************************************************************
 * apps/uav/uorb/topics/vehicle_attitude.hpp
 *
 * Topic: vehicle_attitude - Tư thế (orientation) của UAV
 *
 * PUBLISHER:  estimator_app
 * SUBSCRIBER: state_app, controller (tương lai)
 *
 * MÔ TẢ:
 * - Quaternion biểu diễn rotation từ body → NED
 * - Euler angles để tiện hiển thị
 * - Angular rates từ gyro (đã filter)
 *
 * TẦN SỐ: 250 Hz
 ****************************************************************************/

#pragma once

#include "../orb_defines.hpp"

/**
 * @brief Tư thế vehicle
 *
 * CONVENTION:
 * - Quaternion: [w, x, y, z] Hamilton convention
 * - Euler: ZYX order (yaw → pitch → roll)
 * - Frame: Body to NED transformation
 */
struct vehicle_attitude_s
{
    uint64_t timestamp_us;   ///< Timestamp [µs since boot]

    // Quaternion [w, x, y, z]
    float q[4];              ///< Quaternion (body → NED)

    // Euler angles (tiện cho hiển thị)
    float roll;              ///< Roll [rad] (-π to π)
    float pitch;             ///< Pitch [rad] (-π/2 to π/2)
    float yaw;               ///< Yaw/Heading [rad] (-π to π)

    // Angular rates (filtered)
    float rollspeed;         ///< Roll rate [rad/s]
    float pitchspeed;        ///< Pitch rate [rad/s]
    float yawspeed;          ///< Yaw rate [rad/s]

    uint8_t instance;        ///< Estimator instance
    uint8_t _padding[7];
};

ORB_DECLARE(vehicle_attitude);
