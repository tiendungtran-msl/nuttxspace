/****************************************************************************
 * apps/examples/uav_states_v1/modules/sensors/imu_module.hpp
 *
 * IMU Sensor Module - Sensor Data Acquisition
 *
 * PURPOSE:
 * - Manages multiple ICM-42688-P IMU sensors.
 * - Reads sensor data at fixed rate (PRODUCER in pub/sub).
 * - Publishes raw (bias-corrected) data to uORB topics.
 * - Handles sensor initialization, calibration, health monitoring.
 *
 * ARCHITECTURE:
 * - Single thread owns all SPI transactions -> no bus contention.
 * - Publishes to sensor_imu topic (one per sensor instance).
 * - Consumers (estimator) subscribe to topics independently.
 *
 * FUTURE:
 * - Add magnetometer (BMM150) support.
 * - Add barometer (MS5611) support.
 * - Health monitoring and sensor voting.
 *
 ****************************************************************************/

#pragma once

#include <pthread.h>
#include <cstdint>

#include "../../uorb/uorb.hpp"
#include "../../uorb/topics.hpp"
#include "../../drivers/imu/icm42688p/icm42688p.hpp"
#include "../../platforms/boards/spi_config.h"

namespace modules
{
namespace sensors
{

/****************************************************************************
 * Configuration
 ****************************************************************************/

static constexpr int MAX_IMUS = 4;
static constexpr int SENSOR_RATE_HZ = 100;
static constexpr int CALIBRATION_TIME_MS = 2000;
static constexpr float GRAVITY = 9.80665f;

/****************************************************************************
 * ImuModule - Manages IMU sensors and publishes data
 ****************************************************************************/

class ImuModule
{
public:
    ImuModule();
    ~ImuModule();

    /**
     * @brief Initialize all available IMU sensors
     * @return Number of successfully initialized sensors
     */
    int init();

    /**
     * @brief Calibrate gyro bias (keep UAV still during calibration)
     * @return true if at least one sensor calibrated successfully
     */
    bool calibrate();

    /**
     * @brief Start sensor reading thread (producer)
     * @return 0 on success, negative on error
     */
    int start();

    /**
     * @brief Stop sensor reading thread
     */
    void stop();

    /**
     * @brief Request stop (non-blocking, for signal handlers)
     */
    void request_stop() { _running = false; }

    /**
     * @brief Get topic for specific IMU instance
     */
    uorb::Topic<uorb::sensor_imu_s, 8>& get_topic(int instance)
    {
        return _topics[instance];
    }

    /**
     * @brief Check if sensor is active
     */
    bool is_active(int instance) const
    {
        return instance >= 0 && instance < MAX_IMUS && _sensor_ok[instance];
    }

    /**
     * @brief Get number of active sensors
     */
    int num_active() const { return _num_active; }

private:
    // Thread entry point
    static void* thread_entry(void* arg);
    void run();

    // Sensors
    drivers::imu::ICM42688P* _sensors[MAX_IMUS];
    bool _sensor_ok[MAX_IMUS];
    int _num_active;

    // Topics (one per IMU)
    uorb::Topic<uorb::sensor_imu_s, 8> _topics[MAX_IMUS];

    // Thread
    pthread_t _thread;
    volatile bool _running;
    bool _thread_started;
};

} // namespace sensors
} // namespace modules
