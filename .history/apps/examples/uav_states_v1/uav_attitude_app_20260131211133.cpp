/****************************************************************************
 * apps/examples/uav_states_v1/uav_attitude_app.cpp
 *
 * UAV Attitude Estimation Application
 * 
 * Features:
 * - Support for 1-4 ICM-42688-P IMU sensors
 * - Quaternion-based attitude estimation (PX4-style)
 * - Automatic gyro bias calibration at startup
 * - Clean console output with attitude angles
 * 
 * Hardware conflict avoidance:
 * - Sequential sensor initialization
 * - Single shared SPI bus with proper CS handling
 * - Sensor read ordering to prevent bus contention
 *
 * Architecture note (future-proof, PX4-like):
 * - This app implements a minimal pub/sub (uORB-like) inside one process.
 * - IMU driver task publishes samples to a ring buffer (producer).
 * - Estimator task consumes the latest samples (consumer).
 * - Later, we can split into multiple apps without changing message formats.
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <string.h>
#include <pthread.h>

#include "drivers/imu/icm42688p/icm42688p.hpp"
#include "lib/attitude_estimator/attitude_estimator_q.hpp"
#include "platforms/boards/spi_config.h"
#include "platforms/nuttx/hrt/hrt.h"

using namespace drivers::imu;

/****************************************************************************
 * Configuration
 ****************************************************************************/

#define MAX_IMUS            4
#define LOOP_RATE_HZ        100
#define PRINT_RATE_HZ       10
#define CALIBRATION_TIME_MS 2000
#define IMU_TOPIC_DEPTH     8

static constexpr float GRAVITY = 9.80665f;
static constexpr float RAD2DEG = 57.2957795f;

/****************************************************************************
 * Minimal pub/sub for IMU data (uORB-like, in-process)
 *
 * Purpose:
 * - Decouple sensor I/O from estimation.
 * - Keep timing stable and allow future modules (logger, EKF2, health).
 * - Maintain a clean migration path to multi-app uORB later.
 ****************************************************************************/

struct ImuSample
{
    uint64_t timestamp_us;
    float accel[3];
    float gyro[3];
    float temperature;
    uint8_t instance;
};

struct ImuTopic
{
    pthread_mutex_t lock;
    ImuSample buffer[IMU_TOPIC_DEPTH];
    uint32_t seq;
    uint8_t write_idx;
    bool initialized;
};

static void imu_topic_init(ImuTopic &topic)
{
    pthread_mutex_init(&topic.lock, nullptr);
    topic.seq = 0;
    topic.write_idx = 0;
    topic.initialized = true;
}

static void imu_publish(ImuTopic &topic, const ImuSample &sample)
{
    if (!topic.initialized) {
        imu_topic_init(topic);
    }

    pthread_mutex_lock(&topic.lock);
    topic.buffer[topic.write_idx] = sample;
    topic.write_idx = (topic.write_idx + 1) % IMU_TOPIC_DEPTH;
    topic.seq++;
    pthread_mutex_unlock(&topic.lock);
}

static bool imu_copy_if_updated(ImuTopic &topic, uint32_t &last_seq, ImuSample &out)
{
    if (!topic.initialized) {
        return false;
    }

    pthread_mutex_lock(&topic.lock);
    if (topic.seq == last_seq) {
        pthread_mutex_unlock(&topic.lock);
        return false;
    }

    uint8_t latest_idx = (topic.write_idx == 0) ? (IMU_TOPIC_DEPTH - 1) : (topic.write_idx - 1);
    out = topic.buffer[latest_idx];
    last_seq = topic.seq;
    pthread_mutex_unlock(&topic.lock);
    return true;
}

/****************************************************************************
 * IMU Manager - handles all sensors with proper sequencing
 ****************************************************************************/

class ImuManager
{
public:
    ImuManager() : _num_active(0)
    {
        memset(_sensors, 0, sizeof(_sensors));
        memset(_sensor_ok, 0, sizeof(_sensor_ok));
    }

    ~ImuManager()
    {
        for (int i = 0; i < MAX_IMUS; i++) {
            if (_sensors[i]) {
                delete _sensors[i];
                _sensors[i] = nullptr;
            }
        }
    }

    /**
     * Initialize all available sensors sequentially
     * Returns number of successfully initialized sensors
     */
    int init()
    {
        static const uint32_t devids[MAX_IMUS] = {
            SPIDEV_IMU, SPIDEV_IMU1, SPIDEV_IMU2, SPIDEV_IMU3
        };

        printf("Initializing IMU sensors...\n");
        _num_active = 0;

        for (int i = 0; i < MAX_IMUS; i++) {
            printf("  [IMU%d] ", i);
            fflush(stdout);

            // Create sensor instance
            _sensors[i] = new ICM42688P(NUTTX_SPI_BUS_IMU, devids[i]);
            if (!_sensors[i]) {
                printf("FAILED (allocation)\n");
                continue;
            }

            // Initialize with delay between sensors to avoid bus contention
            int ret = _sensors[i]->initialize();
            if (ret == 0) {
                _sensor_ok[i] = true;
                _num_active++;
                printf("OK\n");
            } else {
                printf("FAILED (error %d)\n", ret);
                delete _sensors[i];
                _sensors[i] = nullptr;
            }

            // Small delay between sensor inits
            usleep(10000);
        }

        printf("Active sensors: %d/%d\n\n", _num_active, MAX_IMUS);
        return _num_active;
    }

    /**
     * Calibrate gyro bias for all sensors (keep UAV still)
     */
    bool calibrate()
    {
        if (_num_active == 0) return false;

        printf("Calibrating (keep still for %d ms)...\n", CALIBRATION_TIME_MS);
        usleep(500000);  // Allow settling

        const int samples = (CALIBRATION_TIME_MS * LOOP_RATE_HZ) / 1000;
        
        for (int i = 0; i < MAX_IMUS; i++) {
            if (!_sensor_ok[i]) continue;

            float gyro_sum[3] = {0, 0, 0};
            float accel_sum[3] = {0, 0, 0};
            int count = 0;

            printf("  [IMU%d] ", i);
            fflush(stdout);

            for (int s = 0; s < samples; s++) {
                ICM42688P::Data data;
                if (_sensors[i]->read(data) == 0) {
                    gyro_sum[0] += data.gyro[0];
                    gyro_sum[1] += data.gyro[1];
                    gyro_sum[2] += data.gyro[2];
                    accel_sum[0] += data.accel[0];
                    accel_sum[1] += data.accel[1];
                    accel_sum[2] += data.accel[2];
                    count++;
                }
                usleep(1000000 / LOOP_RATE_HZ);
            }

            if (count < samples / 2) {
                printf("FAILED (insufficient samples)\n");
                _sensor_ok[i] = false;
                _num_active--;
                continue;
            }

            float inv = 1.0f / count;
            
            // Set gyro bias
            float gyro_bias[3] = {
                gyro_sum[0] * inv,
                gyro_sum[1] * inv,
                gyro_sum[2] * inv
            };
            _sensors[i]->set_gyro_bias(gyro_bias);

            // Calculate accel scale correction (normalize to g)
            float ax = accel_sum[0] * inv;
            float ay = accel_sum[1] * inv;
            float az = accel_sum[2] * inv;
            float norm = sqrtf(ax*ax + ay*ay + az*az);
            float scale = (norm > 1.0f) ? (GRAVITY / norm) : 1.0f;
            _sensors[i]->set_accel_scale_correction(scale);

            printf("OK (bias: %.3f, %.3f, %.3f rad/s)\n",
                   (double)gyro_bias[0], (double)gyro_bias[1], (double)gyro_bias[2]);
        }

        printf("\n");
        return _num_active > 0;
    }

    /**
     * Read data from a specific sensor
     */
    bool read(int index, ICM42688P::Data& data)
    {
        if (index < 0 || index >= MAX_IMUS || !_sensor_ok[index]) {
            return false;
        }
        return _sensors[index]->read(data) == 0;
    }

    int num_active() const { return _num_active; }
    bool is_ok(int index) const { return _sensor_ok[index]; }

private:
    ICM42688P* _sensors[MAX_IMUS];
    bool _sensor_ok[MAX_IMUS];
    int _num_active;
};

/****************************************************************************
 * Application State
 ****************************************************************************/

static volatile bool g_running = true;
static ImuManager g_imu_manager;
static attitude::AttitudeEstimatorQ g_estimator[MAX_IMUS];
static ImuTopic g_imu_topics[MAX_IMUS];
static pthread_t g_imu_thread;

/****************************************************************************
 * Signal Handler
 ****************************************************************************/

static void signal_handler(int signo)
{
    (void)signo;
    g_running = false;
}

/****************************************************************************
 * IMU Producer Thread
 *
 * - Reads sensors at fixed rate and publishes to ring buffer.
 * - Only this thread touches SPI for IMU -> avoids bus contention.
 ****************************************************************************/

static void* imu_producer_thread(void* arg)
{
    (void)arg;

    const uint32_t loop_period_us = 1000000 / LOOP_RATE_HZ;
    uint64_t next_time = hrt_absolute_time();

    while (g_running) {
        for (int i = 0; i < MAX_IMUS; i++) {
            if (!g_imu_manager.is_ok(i)) {
                continue;
            }

            ICM42688P::Data data;
            if (g_imu_manager.read(i, data)) {
                ImuSample sample{};
                sample.timestamp_us = data.timestamp_us;
                sample.accel[0] = data.accel[0];
                sample.accel[1] = data.accel[1];
                sample.accel[2] = data.accel[2];
                sample.gyro[0] = data.gyro[0];
                sample.gyro[1] = data.gyro[1];
                sample.gyro[2] = data.gyro[2];
                sample.temperature = data.temperature;
                sample.instance = static_cast<uint8_t>(i);

                imu_publish(g_imu_topics[i], sample);
            }
        }

        next_time += loop_period_us;
        int64_t sleep_us = (int64_t)next_time - (int64_t)hrt_absolute_time();
        if (sleep_us > 0) {
            usleep(sleep_us);
        } else {
            next_time = hrt_absolute_time();
        }
    }

    return nullptr;
}

/****************************************************************************
 * Print attitude
 ****************************************************************************/

static void print_header(void)
{
    printf("%-8s", "Time");
    for (int i = 0; i < MAX_IMUS; i++) {
        if (g_imu_manager.is_ok(i)) {
            printf("  IMU%d Roll   Pitch    Yaw ", i);
        }
    }
    printf("\n");

    printf("--------");
    for (int i = 0; i < MAX_IMUS; i++) {
        if (g_imu_manager.is_ok(i)) {
            printf("  ---------------------------");
        }
    }
    printf("\n");
}

static void print_attitude(uint32_t elapsed_sec, int loop_count)
{
    printf("%5lu.%01lus", 
           (unsigned long)elapsed_sec,
           (unsigned long)((loop_count / (LOOP_RATE_HZ / PRINT_RATE_HZ)) % 10));

    for (int i = 0; i < MAX_IMUS; i++) {
        if (!g_imu_manager.is_ok(i)) continue;

        attitude::EulerAngles euler = g_estimator[i].get_euler();
        printf("  %+7.1f %+7.1f %+7.1f",
               (double)(euler.roll * RAD2DEG),
               (double)(euler.pitch * RAD2DEG),
               (double)(euler.yaw * RAD2DEG));
    }
    printf("\n");
}

/****************************************************************************
 * Main
 ****************************************************************************/

extern "C" int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    printf("\n");
    printf("========================================\n");
    printf("  UAV Attitude Estimation\n");
    printf("  ICM-42688-P IMU @ %d Hz\n", LOOP_RATE_HZ);
    printf("========================================\n\n");

    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize sensors
    if (g_imu_manager.init() == 0) {
        printf("ERROR: No sensors available!\n");
        return 1;
    }

    // Calibrate
    if (!g_imu_manager.calibrate()) {
        printf("ERROR: Calibration failed!\n");
        return 1;
    }

    // Initialize attitude estimators
    for (int i = 0; i < MAX_IMUS; i++) {
        if (g_imu_manager.is_ok(i)) {
            g_estimator[i].reset();
        }
    }

    // Initialize IMU topics and start producer thread
    for (int i = 0; i < MAX_IMUS; i++) {
        imu_topic_init(g_imu_topics[i]);
    }

    if (pthread_create(&g_imu_thread, nullptr, imu_producer_thread, nullptr) != 0) {
        printf("ERROR: Failed to start IMU producer thread!\n");
        return 1;
    }

    printf("Running at %d Hz. Press Ctrl+C to stop.\n\n", LOOP_RATE_HZ);
    print_header();

    // Main loop timing
    const int print_divider = LOOP_RATE_HZ / PRINT_RATE_HZ;

    uint64_t last_ts[MAX_IMUS] = {0};
    uint32_t last_seq[MAX_IMUS] = {0};
    uint32_t loop_count = 0;
    uint32_t start_time = hrt_absolute_time() / 1000000;

    while (g_running) {
        // Consume latest samples (producer->consumer)
        for (int i = 0; i < MAX_IMUS; i++) {
            if (!g_imu_manager.is_ok(i)) continue;

            ImuSample sample;
            if (imu_copy_if_updated(g_imu_topics[i], last_seq[i], sample)) {
                float dt = 1.0f / LOOP_RATE_HZ;
                if (last_ts[i] > 0 && sample.timestamp_us > last_ts[i]) {
                    dt = (sample.timestamp_us - last_ts[i]) * 1e-6f;
                }
                last_ts[i] = sample.timestamp_us;

                g_estimator[i].update(sample.accel, sample.gyro, dt);
            }
        }

        // Print at lower rate
        loop_count++;
        if (loop_count % print_divider == 0) {
            uint32_t elapsed = (hrt_absolute_time() / 1000000) - start_time;
            print_attitude(elapsed, loop_count);
        }

        // Consumer loop does not enforce hard timing. Producer thread is timing-critical.
        usleep(1000);
    }

    // Stop producer thread cleanly
    pthread_join(g_imu_thread, nullptr);

    printf("\nStopping...\n");
    printf("Done.\n");
    
    return 0;
}
