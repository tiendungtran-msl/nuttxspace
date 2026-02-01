/****************************************************************************
 * apps/examples/uav_states_v1/lib/orb/topics.hpp
 *
 * ORB Topic Definitions - Message structures for pub-sub
 ****************************************************************************/

#pragma once

#include "orb.hpp"

namespace orb
{

//=============================================================================
// Sensor Messages
//=============================================================================

/**
 * @brief Raw IMU data (accelerometer + gyroscope)
 */
struct sensor_imu_s : public MessageBase
{
    float accel_m_s2[3];    ///< Accelerometer (m/s²)
    float gyro_rad_s[3];    ///< Gyroscope (rad/s)
    float temperature_c;    ///< Temperature (°C)
    
    sensor_imu_s() : temperature_c(0.0f) 
    {
        accel_m_s2[0] = accel_m_s2[1] = accel_m_s2[2] = 0.0f;
        gyro_rad_s[0] = gyro_rad_s[1] = gyro_rad_s[2] = 0.0f;
    }
};

/**
 * @brief Calibrated accelerometer
 */
struct sensor_accel_s : public MessageBase
{
    float x;        ///< X acceleration (m/s²)
    float y;        ///< Y acceleration (m/s²)
    float z;        ///< Z acceleration (m/s²)
    float temperature;
    uint32_t error_count;
    
    sensor_accel_s() : x(0), y(0), z(0), temperature(0), error_count(0) {}
};

/**
 * @brief Calibrated gyroscope
 */
struct sensor_gyro_s : public MessageBase
{
    float x;        ///< X angular rate (rad/s)
    float y;        ///< Y angular rate (rad/s)
    float z;        ///< Z angular rate (rad/s)
    float temperature;
    uint32_t error_count;
    
    sensor_gyro_s() : x(0), y(0), z(0), temperature(0), error_count(0) {}
};

//=============================================================================
// Attitude Messages
//=============================================================================

/**
 * @brief Vehicle attitude in quaternion form
 */
struct vehicle_attitude_s : public MessageBase
{
    float q[4];             ///< Quaternion [w, x, y, z]
    float rollspeed;        ///< Roll rate (rad/s)
    float pitchspeed;       ///< Pitch rate (rad/s)
    float yawspeed;         ///< Yaw rate (rad/s)
    float delta_q_reset[4]; ///< Quaternion delta from last reset
    uint8_t quat_reset_counter;
    
    vehicle_attitude_s() 
        : rollspeed(0), pitchspeed(0), yawspeed(0), quat_reset_counter(0)
    {
        q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
        delta_q_reset[0] = 1.0f;
        delta_q_reset[1] = delta_q_reset[2] = delta_q_reset[3] = 0.0f;
    }
};

/**
 * @brief Euler angles (for debugging/display)
 */
struct vehicle_euler_s : public MessageBase
{
    float roll;         ///< Roll (rad)
    float pitch;        ///< Pitch (rad)
    float yaw;          ///< Yaw (rad)
    float roll_deg;     ///< Roll (degrees)
    float pitch_deg;    ///< Pitch (degrees)
    float yaw_deg;      ///< Yaw (degrees)
    float gyro_bias[3]; ///< Estimated gyro bias (rad/s)
    bool valid;
    
    vehicle_euler_s() 
        : roll(0), pitch(0), yaw(0)
        , roll_deg(0), pitch_deg(0), yaw_deg(0)
        , valid(false) 
    {
        gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0.0f;
    }
};

//=============================================================================
// Estimator Status
//=============================================================================

/**
 * @brief State estimator status and health
 */
struct estimator_status_s : public MessageBase
{
    float gyro_bias[3];         ///< Estimated gyro bias (rad/s)
    float accel_bias[3];        ///< Estimated accel bias (m/s²)
    float attitude_variance;    ///< Attitude uncertainty
    uint32_t solution_status;   ///< Status flags
    bool attitude_valid;
    bool velocity_valid;
    bool position_valid;
    
    estimator_status_s() 
        : attitude_variance(0)
        , solution_status(0)
        , attitude_valid(false)
        , velocity_valid(false)
        , position_valid(false)
    {
        gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0.0f;
        accel_bias[0] = accel_bias[1] = accel_bias[2] = 0.0f;
    }
};

//=============================================================================
// Topic Instances (Global)
//=============================================================================

// Topic declarations (defined in topics.cpp)
// Uses ORB_MULTI_MAX_INSTANCES from orb.hpp
extern Topic<sensor_imu_s>          topic_sensor_imu[ORB_MULTI_MAX_INSTANCES];
extern Topic<sensor_accel_s>        topic_sensor_accel[ORB_MULTI_MAX_INSTANCES];
extern Topic<sensor_gyro_s>         topic_sensor_gyro[ORB_MULTI_MAX_INSTANCES];
extern Topic<vehicle_attitude_s>    topic_vehicle_attitude[ORB_MULTI_MAX_INSTANCES];
extern Topic<vehicle_euler_s>       topic_vehicle_euler[ORB_MULTI_MAX_INSTANCES];
extern Topic<estimator_status_s>    topic_estimator_status[ORB_MULTI_MAX_INSTANCES];

} // namespace orb
