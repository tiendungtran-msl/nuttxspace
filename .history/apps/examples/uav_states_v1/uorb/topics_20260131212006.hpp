/****************************************************************************
 * apps/examples/uav_states_v1/uorb/topics.hpp
 *
 * Message Definitions for uORB Topics
 *
 * PURPOSE:
 * - Define data structures exchanged between modules.
 * - Each struct represents one "message type" in pub/sub.
 * - Keep structures POD (Plain Old Data) for easy copy.
 *
 * NAMING CONVENTION (PX4-like):
 * - sensor_*    : Raw sensor data
 * - vehicle_*   : Processed vehicle state
 * - estimator_* : Internal estimator states
 *
 * FUTURE EXPANSION:
 * - Add mag_sample_s for magnetometer
 * - Add baro_sample_s for barometer  
 * - Add gps_sample_s for GPS
 * - Add vehicle_local_position_s for EKF2 output
 *
 ****************************************************************************/

#pragma once

#include <cstdint>

namespace uorb
{

/****************************************************************************
 * sensor_imu_s - IMU sensor sample
 *
 * Published by: modules/sensors (imu_manager)
 * Consumed by: modules/estimator (attitude_estimator)
 *
 * Contains raw (bias-corrected) accelerometer and gyroscope data
 * from ICM-42688-P or similar 6-axis IMU.
 ****************************************************************************/

struct sensor_imu_s
{
    uint64_t timestamp_us;   // Sample timestamp (microseconds since boot)
    float accel[3];          // Acceleration [m/s²] (X, Y, Z in body frame)
    float gyro[3];           // Angular rate [rad/s] (X, Y, Z in body frame)
    float temperature;       // Die temperature [°C]
    uint8_t instance;        // Sensor instance (0-3 for multi-IMU)
    uint8_t _padding[3];     // Alignment padding
};

/****************************************************************************
 * vehicle_attitude_s - Vehicle attitude (orientation)
 *
 * Published by: modules/estimator
 * Consumed by: main (display), future flight controller
 *
 * Quaternion representation of vehicle orientation.
 * Also includes Euler angles for convenience (computed from quaternion).
 ****************************************************************************/

struct vehicle_attitude_s
{
    uint64_t timestamp_us;   // Timestamp when attitude was computed
    float q[4];              // Quaternion [w, x, y, z] (NED frame)
    float roll;              // Roll angle [rad] (-π to π)
    float pitch;             // Pitch angle [rad] (-π/2 to π/2)
    float yaw;               // Yaw angle [rad] (-π to π)
    float rollspeed;         // Angular velocity around X [rad/s]
    float pitchspeed;        // Angular velocity around Y [rad/s]
    float yawspeed;          // Angular velocity around Z [rad/s]
    uint8_t instance;        // Estimator instance (matches IMU instance)
    uint8_t _padding[7];     // Alignment padding
};

/****************************************************************************
 * estimator_status_s - Estimator health & diagnostics
 *
 * Published by: modules/estimator
 * Consumed by: main (health monitoring), logger
 *
 * Useful for debugging and flight recorder.
 ****************************************************************************/

struct estimator_status_s
{
    uint64_t timestamp_us;
    float gyro_bias[3];      // Estimated gyro bias [rad/s]
    float accel_bias[3];     // Estimated accel bias [m/s²]
    float dt;                // Last update interval [s]
    uint32_t update_count;   // Number of estimator updates
    uint8_t instance;
    bool attitude_valid;     // true if attitude is trustworthy
    uint8_t _padding[2];
};

/****************************************************************************
 * sensor_calibration_s - Calibration parameters
 *
 * Used internally for sensor correction.
 * Can be saved to/loaded from flash.
 ****************************************************************************/

struct sensor_calibration_s
{
    float gyro_bias[3];      // Gyro bias [rad/s]
    float accel_bias[3];     // Accel bias [m/s²]
    float accel_scale[3];    // Accel scale factors
    float rotation[9];       // Rotation matrix (row-major)
    uint32_t calibration_id; // Unique ID for calibration
    bool valid;              // true if calibration is valid
    uint8_t _padding[3];
};

} // namespace uorb
