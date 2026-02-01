/****************************************************************************
 * apps/uav/uorb/topics/ekf2_status.hpp
 *
 * Topic: ekf2_status - Trạng thái nội bộ EKF2
 *
 * PUBLISHER:  estimator_app
 * SUBSCRIBER: state_app (health monitoring), logger
 *
 * MÔ TẢ:
 * - Bias estimates
 * - Innovation test ratios
 * - Fusion status flags
 * - Hữu ích cho debug và tuning
 *
 * TẦN SỐ: 10-25 Hz
 ****************************************************************************/

#pragma once

#include "../orb_defines.hpp"

/**
 * @brief Trạng thái EKF2
 */
struct ekf2_status_s
{
    uint64_t timestamp_us;   ///< Timestamp [µs since boot]

    // Bias estimates
    float gyro_bias[3];      ///< Gyro bias estimate [rad/s]
    float accel_bias[3];     ///< Accel bias estimate [m/s²]

    // Innovation test ratios (chi-square / threshold)
    // < 1.0 = normal, > 1.0 = innovation bị reject
    float gps_hpos_test_ratio;
    float gps_vpos_test_ratio;
    float gps_vel_test_ratio;
    float baro_hgt_test_ratio;
    float mag_heading_test_ratio;

    // Covariance diagonals (uncertainty)
    float pos_var[3];        ///< Position variance [m²]
    float vel_var[3];        ///< Velocity variance [(m/s)²]

    // Control status flags
    bool tilt_align;         ///< Tilt (roll/pitch) aligned
    bool yaw_align;          ///< Yaw aligned
    bool gps_fused;          ///< GPS đang được fusion
    bool baro_fused;         ///< Baro đang được fusion
    bool mag_fused;          ///< Mag heading đang được fusion

    uint8_t _padding[3];
};

ORB_DECLARE(ekf2_status);
