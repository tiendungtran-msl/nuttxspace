/****************************************************************************
 * apps/examples/uav_states_v1/modules/estimator/attitude_module.cpp
 *
 * Attitude Estimator Module Implementation
 *
 * This module:
 * 1. Subscribes to IMU data from ImuModule via uORB topics.
 * 2. Runs quaternion-based complementary filter (PX4-style).
 * 3. Publishes vehicle_attitude for display/control.
 *
 * Each update() call:
 * - Checks all IMU subscriptions for new data.
 * - For each new sample, runs estimator update.
 * - Publishes updated attitude.
 *
 * Called from main loop (not a separate thread) to keep things simple.
 * The main loop controls the update rate.
 *
 ****************************************************************************/

#include "attitude_module.hpp"
#include "../../platforms/nuttx/hrt/hrt.h"

#include <syslog.h>
#include <cstring>

namespace modules
{
namespace estimator
{

/****************************************************************************
 * Constructor
 ****************************************************************************/

AttitudeModule::AttitudeModule()
    : _num_estimators(0)
{
    memset(_imu_subs, 0, sizeof(_imu_subs));
    memset(_imu_topics, 0, sizeof(_imu_topics));
    memset(_last_imu_timestamp, 0, sizeof(_last_imu_timestamp));
    memset(_update_count, 0, sizeof(_update_count));
}

/****************************************************************************
 * init - Initialize estimators and create subscriptions
 ****************************************************************************/

void AttitudeModule::init(uorb::Topic<uorb::sensor_imu_s, 8>* imu_topics, int num_imus)
{
    _num_estimators = (num_imus > MAX_ESTIMATORS) ? MAX_ESTIMATORS : num_imus;

    for (int i = 0; i < _num_estimators; i++) {
        // Store topic pointer and create subscription
        _imu_topics[i] = &imu_topics[i];
        _imu_subs[i] = new uorb::Subscription<uorb::sensor_imu_s, 8>(imu_topics[i]);

        // Initialize estimator
        _estimators[i].reset();

        // Initialize output topics
        _att_topics[i].init();
        _status_topics[i].init();

        _last_imu_timestamp[i] = 0;
        _update_count[i] = 0;
    }

    syslog(LOG_INFO, "[estimator] Initialized %d attitude estimators\n", _num_estimators);
}

/****************************************************************************
 * update - Process one cycle
 *
 * For each IMU with new data:
 * 1. Compute dt from timestamps
 * 2. Update quaternion estimator
 * 3. Publish vehicle_attitude
 ****************************************************************************/

int AttitudeModule::update()
{
    int updated = 0;

    for (int i = 0; i < _num_estimators; i++) {
        if (!_imu_subs[i]) {
            continue;
        }

        uorb::sensor_imu_s imu;
        if (!_imu_subs[i]->copy(imu)) {
            continue;
        }

        // Compute dt from timestamps
        float dt = 0.01f;  // Default 100Hz
        if (_last_imu_timestamp[i] > 0 && imu.timestamp_us > _last_imu_timestamp[i]) {
            dt = (imu.timestamp_us - _last_imu_timestamp[i]) * 1e-6f;
            // Clamp dt to reasonable range
            if (dt < 0.001f) dt = 0.001f;
            if (dt > 0.1f) dt = 0.1f;
        }
        _last_imu_timestamp[i] = imu.timestamp_us;

        // Update estimator
        bool valid = _estimators[i].update(imu.accel, imu.gyro, dt);

        // Pack and publish vehicle_attitude
        attitude::EulerAngles euler = _estimators[i].get_euler();
        attitude::Quatf q = _estimators[i].get_quaternion();
        attitude::Vector3f rates = _estimators[i].get_rates();

        uorb::vehicle_attitude_s att{};
        att.timestamp_us = hrt_absolute_time();
        att.q[0] = q.w;
        att.q[1] = q.x;
        att.q[2] = q.y;
        att.q[3] = q.z;
        att.roll = euler.roll;
        att.pitch = euler.pitch;
        att.yaw = euler.yaw;
        att.rollspeed = rates.x;
        att.pitchspeed = rates.y;
        att.yawspeed = rates.z;
        att.instance = static_cast<uint8_t>(i);

        _att_topics[i].publish(att);

        // Publish estimator status (for debugging/logging)
        _update_count[i]++;
        if ((_update_count[i] % 100) == 0) {
            // Publish status every 1 second @ 100Hz
            attitude::Vector3f bias = _estimators[i].get_gyro_bias();

            uorb::estimator_status_s status{};
            status.timestamp_us = att.timestamp_us;
            status.gyro_bias[0] = bias.x;
            status.gyro_bias[1] = bias.y;
            status.gyro_bias[2] = bias.z;
            status.dt = dt;
            status.update_count = _update_count[i];
            status.instance = static_cast<uint8_t>(i);
            status.attitude_valid = valid;

            _status_topics[i].publish(status);
        }

        updated++;
    }

    return updated;
}

/****************************************************************************
 * get_attitude - Get latest attitude (for main to display)
 ****************************************************************************/

bool AttitudeModule::get_attitude(int instance, uorb::vehicle_attitude_s& att)
{
    if (instance < 0 || instance >= _num_estimators) {
        return false;
    }
    return _att_topics[instance].copy(att);
}

} // namespace estimator
} // namespace modules
