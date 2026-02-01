/****************************************************************************
 * apps/examples/uav_states_v1/modules/estimator/attitude_module.hpp
 *
 * Attitude Estimator Module - Quaternion-based Attitude Estimation
 *
 * PURPOSE:
 * - Consumes IMU data from sensor_imu topics (CONSUMER).
 * - Computes attitude using complementary filter with quaternions.
 * - Publishes vehicle_attitude topics (PRODUCER).
 *
 * ARCHITECTURE:
 * - One estimator instance per IMU for redundancy.
 * - Subscribes to sensor_imu topics from ImuModule.
 * - Publishes vehicle_attitude topics for display/controller.
 * - Future: sensor voting, EKF2 integration.
 *
 * DATA FLOW:
 *   ImuModule (sensor_imu) --> AttitudeModule --> (vehicle_attitude) --> Main
 *
 ****************************************************************************/

#pragma once

#include <cstdint>

#include "../../uorb/uorb.hpp"
#include "../../uorb/topics.hpp"
#include "../../lib/attitude_estimator/attitude_estimator_q.hpp"

namespace modules
{
namespace estimator
{

/****************************************************************************
 * Configuration
 ****************************************************************************/

static constexpr int MAX_ESTIMATORS = 4;  // Match MAX_IMUS

/****************************************************************************
 * AttitudeModule - Computes attitude from IMU data
 ****************************************************************************/

class AttitudeModule
{
public:
    AttitudeModule();
    ~AttitudeModule() = default;

    /**
     * @brief Initialize estimators and subscribe to IMU topics
     * @param imu_topics Array of IMU topic pointers (from ImuModule)
     * @param num_imus Number of active IMUs
     */
    void init(uorb::Topic<uorb::sensor_imu_s, 8>* imu_topics, int num_imus);

    /**
     * @brief Process one update cycle
     *
     * Called from main loop. Checks for new IMU data, updates estimators,
     * and publishes attitude.
     *
     * @return Number of estimators updated
     */
    int update();

    /**
     * @brief Get attitude topic for specific instance
     */
    uorb::Topic<uorb::vehicle_attitude_s, 4>& get_topic(int instance)
    {
        return _att_topics[instance];
    }

    /**
     * @brief Get estimator status topic
     */
    uorb::Topic<uorb::estimator_status_s, 2>& get_status_topic(int instance)
    {
        return _status_topics[instance];
    }

    /**
     * @brief Get latest attitude (convenience for main)
     */
    bool get_attitude(int instance, uorb::vehicle_attitude_s& att);

    /**
     * @brief Check if instance is valid
     */
    bool is_valid(int instance) const
    {
        return instance >= 0 && instance < _num_estimators;
    }

    /**
     * @brief Get number of estimators
     */
    int num_estimators() const { return _num_estimators; }

private:
    // IMU subscriptions (pointers to ImuModule's topics)
    uorb::Subscription<uorb::sensor_imu_s, 8>* _imu_subs[MAX_ESTIMATORS];
    uorb::Topic<uorb::sensor_imu_s, 8>* _imu_topics[MAX_ESTIMATORS];

    // Estimator instances
    attitude::AttitudeEstimatorQ _estimators[MAX_ESTIMATORS];
    uint64_t _last_imu_timestamp[MAX_ESTIMATORS];

    // Output topics
    uorb::Topic<uorb::vehicle_attitude_s, 4> _att_topics[MAX_ESTIMATORS];
    uorb::Topic<uorb::estimator_status_s, 2> _status_topics[MAX_ESTIMATORS];

    // State
    int _num_estimators;
    uint32_t _update_count[MAX_ESTIMATORS];
};

} // namespace estimator
} // namespace modules
