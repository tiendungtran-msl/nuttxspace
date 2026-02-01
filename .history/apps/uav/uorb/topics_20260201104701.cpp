/****************************************************************************
 * apps/uav/uorb/topics.cpp
 *
 * Topic Definitions - ORB_DEFINE cho tất cả topics
 *
 * MỤC ĐÍCH:
 * - Mỗi topic cần một ORB_DEFINE để tạo metadata
 * - Tập trung tất cả defines vào một file
 * - Tránh duplicate definition
 *
 ****************************************************************************/

#include "orb_defines.hpp"

// Sensor topics
#include "topics/sensor_imu.hpp"
#include "topics/sensor_mag.hpp"
#include "topics/sensor_baro.hpp"
#include "topics/sensor_gps.hpp"

// Vehicle topics
#include "topics/vehicle_attitude.hpp"
#include "topics/vehicle_local_position.hpp"
#include "topics/vehicle_state.hpp"

// Estimator topics
#include "topics/ekf2_status.hpp"

//=============================================================================
// Sensor Topics
//=============================================================================

ORB_DEFINE(sensor_imu, sensor_imu_s);
ORB_DEFINE(sensor_mag, sensor_mag_s);
ORB_DEFINE(sensor_baro, sensor_baro_s);
ORB_DEFINE(sensor_gps, sensor_gps_s);

//=============================================================================
// Vehicle Topics
//=============================================================================

ORB_DEFINE(vehicle_attitude, vehicle_attitude_s);
ORB_DEFINE(vehicle_local_position, vehicle_local_position_s);
ORB_DEFINE(vehicle_state, vehicle_state_s);

//=============================================================================
// Estimator Topics
//=============================================================================

ORB_DEFINE(ekf2_status, ekf2_status_s);
