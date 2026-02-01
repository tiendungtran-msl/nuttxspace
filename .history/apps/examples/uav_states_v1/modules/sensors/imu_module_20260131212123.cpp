/****************************************************************************
 * apps/examples/uav_states_v1/modules/sensors/imu_module.cpp
 *
 * IMU Sensor Module Implementation
 *
 * This module:
 * 1. Initializes up to 4 ICM-42688-P sensors sequentially.
 * 2. Calibrates gyro bias at startup (user must keep UAV still).
 * 3. Runs a producer thread that reads sensors and publishes to uORB.
 *
 * The producer thread is the ONLY code touching SPI for IMU, preventing
 * bus contention that would occur with multiple threads.
 *
 ****************************************************************************/

#include "imu_module.hpp"
#include "../../platforms/nuttx/hrt/hrt.h"

#include <nuttx/config.h>
#include <syslog.h>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <unistd.h>

namespace modules
{
namespace sensors
{

/****************************************************************************
 * Constructor / Destructor
 ****************************************************************************/

ImuModule::ImuModule()
    : _num_active(0)
    , _running(false)
    , _thread_started(false)
{
    memset(_sensors, 0, sizeof(_sensors));
    memset(_sensor_ok, 0, sizeof(_sensor_ok));
}

ImuModule::~ImuModule()
{
    stop();
    for (int i = 0; i < MAX_IMUS; i++) {
        if (_sensors[i]) {
            delete _sensors[i];
            _sensors[i] = nullptr;
        }
    }
}

/****************************************************************************
 * init - Initialize all available IMU sensors
 *
 * Sequential initialization with delays to avoid SPI bus contention.
 * Each sensor gets a unique chip select.
 ****************************************************************************/

int ImuModule::init()
{
    // Device IDs for each IMU (defined in spi_config.h)
    static const uint32_t devids[MAX_IMUS] = {
        SPIDEV_IMU, SPIDEV_IMU1, SPIDEV_IMU2, SPIDEV_IMU3
    };

    syslog(LOG_INFO, "[sensors] Initializing IMU sensors...\n");
    _num_active = 0;

    for (int i = 0; i < MAX_IMUS; i++) {
        syslog(LOG_INFO, "[sensors]   IMU%d: ", i);

        // Create sensor instance
        _sensors[i] = new drivers::imu::ICM42688P(NUTTX_SPI_BUS_IMU, devids[i]);
        if (!_sensors[i]) {
            syslog(LOG_ERR, "allocation failed\n");
            continue;
        }

        // Initialize sensor
        int ret = _sensors[i]->initialize();
        if (ret == 0) {
            _sensor_ok[i] = true;
            _num_active++;
            syslog(LOG_INFO, "OK\n");
        } else {
            syslog(LOG_ERR, "init failed (%d)\n", ret);
            delete _sensors[i];
            _sensors[i] = nullptr;
        }

        // Delay between sensor inits to avoid bus contention
        usleep(10000);
    }

    syslog(LOG_INFO, "[sensors] Active sensors: %d/%d\n", _num_active, MAX_IMUS);

    // Initialize uORB topics
    for (int i = 0; i < MAX_IMUS; i++) {
        _topics[i].init();
    }

    return _num_active;
}

/****************************************************************************
 * calibrate - Calibrate gyro bias
 *
 * Collects samples while UAV is stationary and computes average bias.
 * Also calculates accelerometer scale correction (should read 1g when still).
 ****************************************************************************/

bool ImuModule::calibrate()
{
    if (_num_active == 0) {
        return false;
    }

    syslog(LOG_INFO, "[sensors] Calibrating (keep still for %d ms)...\n",
           CALIBRATION_TIME_MS);

    // Allow sensor settling
    usleep(500000);

    const int samples = (CALIBRATION_TIME_MS * SENSOR_RATE_HZ) / 1000;

    for (int i = 0; i < MAX_IMUS; i++) {
        if (!_sensor_ok[i]) continue;

        float gyro_sum[3] = {0, 0, 0};
        float accel_sum[3] = {0, 0, 0};
        int count = 0;

        syslog(LOG_INFO, "[sensors]   IMU%d: ", i);

        for (int s = 0; s < samples; s++) {
            drivers::imu::ICM42688P::Data data;
            if (_sensors[i]->read(data) == 0) {
                gyro_sum[0] += data.gyro[0];
                gyro_sum[1] += data.gyro[1];
                gyro_sum[2] += data.gyro[2];
                accel_sum[0] += data.accel[0];
                accel_sum[1] += data.accel[1];
                accel_sum[2] += data.accel[2];
                count++;
            }
            usleep(1000000 / SENSOR_RATE_HZ);
        }

        if (count < samples / 2) {
            syslog(LOG_ERR, "insufficient samples\n");
            _sensor_ok[i] = false;
            _num_active--;
            continue;
        }

        float inv = 1.0f / count;

        // Compute and set gyro bias
        float gyro_bias[3] = {
            gyro_sum[0] * inv,
            gyro_sum[1] * inv,
            gyro_sum[2] * inv
        };
        _sensors[i]->set_gyro_bias(gyro_bias);

        // Compute accel scale correction
        float ax = accel_sum[0] * inv;
        float ay = accel_sum[1] * inv;
        float az = accel_sum[2] * inv;
        float norm = sqrtf(ax*ax + ay*ay + az*az);
        float scale = (norm > 1.0f) ? (GRAVITY / norm) : 1.0f;
        _sensors[i]->set_accel_scale_correction(scale);

        syslog(LOG_INFO, "OK (bias: %.3f, %.3f, %.3f rad/s)\n",
               (double)gyro_bias[0], (double)gyro_bias[1], (double)gyro_bias[2]);
    }

    return _num_active > 0;
}

/****************************************************************************
 * start / stop - Thread management
 ****************************************************************************/

int ImuModule::start()
{
    if (_thread_started) {
        return 0;
    }

    _running = true;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 4096);

    // Higher priority for sensor reading (time-critical)
    struct sched_param param;
    param.sched_priority = SCHED_PRIORITY_DEFAULT + 10;
    pthread_attr_setschedparam(&attr, &param);

    int ret = pthread_create(&_thread, &attr, thread_entry, this);
    pthread_attr_destroy(&attr);

    if (ret != 0) {
        syslog(LOG_ERR, "[sensors] Failed to create thread: %d\n", ret);
        _running = false;
        return -ret;
    }

    _thread_started = true;
    syslog(LOG_INFO, "[sensors] Sensor thread started @ %d Hz\n", SENSOR_RATE_HZ);
    return 0;
}

void ImuModule::stop()
{
    if (!_thread_started) {
        return;
    }

    _running = false;
    pthread_join(_thread, nullptr);
    _thread_started = false;
    syslog(LOG_INFO, "[sensors] Sensor thread stopped\n");
}

/****************************************************************************
 * Thread - Sensor reading loop
 *
 * Runs at SENSOR_RATE_HZ, reads all active sensors, publishes to topics.
 * Uses hrt_absolute_time() for precise timing.
 ****************************************************************************/

void* ImuModule::thread_entry(void* arg)
{
    ImuModule* self = static_cast<ImuModule*>(arg);
    self->run();
    return nullptr;
}

void ImuModule::run()
{
    const uint32_t loop_period_us = 1000000 / SENSOR_RATE_HZ;
    uint64_t next_time = hrt_absolute_time();

    while (_running) {
        // Read all sensors sequentially
        for (int i = 0; i < MAX_IMUS; i++) {
            if (!_sensor_ok[i]) {
                continue;
            }

            drivers::imu::ICM42688P::Data data;
            if (_sensors[i]->read(data) == 0) {
                // Pack into uORB message
                uorb::sensor_imu_s msg{};
                msg.timestamp_us = data.timestamp_us;
                msg.accel[0] = data.accel[0];
                msg.accel[1] = data.accel[1];
                msg.accel[2] = data.accel[2];
                msg.gyro[0] = data.gyro[0];
                msg.gyro[1] = data.gyro[1];
                msg.gyro[2] = data.gyro[2];
                msg.temperature = data.temperature;
                msg.instance = static_cast<uint8_t>(i);

                // Publish to topic
                _topics[i].publish(msg);
            }
        }

        // Precise timing control
        next_time += loop_period_us;
        int64_t sleep_us = (int64_t)next_time - (int64_t)hrt_absolute_time();
        if (sleep_us > 0) {
            usleep(sleep_us);
        } else {
            // Overrun - reset timing baseline
            next_time = hrt_absolute_time();
        }
    }
}

} // namespace sensors
} // namespace modules
