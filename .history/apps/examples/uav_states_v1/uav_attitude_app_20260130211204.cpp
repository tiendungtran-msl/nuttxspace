/****************************************************************************
 * apps/examples/uav_states_v1/uav_attitude_app.cpp
 *
 * UAV Attitude Application - NuttX RTOS Architecture
 * 
 * Proper multi-tasking with NuttX work queues and pub-sub:
 * 
 *   [Sensor Task]     - High priority, reads IMUs @ 400Hz
 *         ↓ publish sensor_imu topic
 *   [Estimator Task]  - Medium priority, computes attitude @ 100Hz
 *         ↓ publish vehicle_attitude topic
 *   [Console Task]    - Low priority, prints status @ low rate
 *         ↓ subscribe to vehicle_euler topic
 * 
 * Key design principles:
 * - NO printf in high-rate tasks (causes UART blocking)
 * - Avoid opening/configuring UART from app (do not touch console UART)
 * - Use pthread with proper priorities
 * - ORB pub-sub for thread-safe data sharing
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <math.h>
#include <errno.h>

#include "drivers/imu/icm42688p/icm42688p.hpp"
#include "lib/orb/orb.hpp"
#include "lib/orb/topics.hpp"
#include "lib/attitude_estimator/attitude_estimator_q.hpp"
#include "calibration/sensor_calibration.hpp"
#include "platforms/boards/spi_config.h"
#include "platforms/nuttx/hrt/hrt.h"

using namespace drivers::imu;

//=============================================================================
// Configuration
//=============================================================================

#define NUM_IMUS            4

// Task rates (Hz)
#define SENSOR_RATE_HZ      400
#define ESTIMATOR_RATE_HZ   100

// Console output rate (keep low to avoid UART blocking)
#define CONSOLE_RATE_HZ     2

// Task priorities (higher = more important)
// NuttX default priority is 100, max is typically 255
#define SENSOR_PRIORITY     120   // Highest - time critical
#define ESTIMATOR_PRIORITY  110   // Medium - compute
#define CONSOLE_PRIORITY    90    // Lowest - can be delayed

// Task stack sizes
#define SENSOR_STACK_SIZE     4096
#define ESTIMATOR_STACK_SIZE  8192
#define CONSOLE_STACK_SIZE    4096

// Calibration
#define CALIBRATION_SAMPLES  400    // 1 second at 400Hz
#define CALIBRATION_SETTLE_MS 500   // Wait before calibration

//=============================================================================
// Global State
//=============================================================================

static volatile bool g_running = true;
static volatile bool g_calibrated = false;

// Sensors
static ICM42688P* g_sensors[NUM_IMUS] = {nullptr};
static bool g_sensor_ok[NUM_IMUS] = {false};
static int g_active_sensors = 0;

static const uint32_t g_sensor_devids[NUM_IMUS] = {
    SPIDEV_IMU, SPIDEV_IMU1, SPIDEV_IMU2, SPIDEV_IMU3
};

// Calibration data
static calibration::Gyroscope g_gyro_cal[NUM_IMUS];
static calibration::Accelerometer g_accel_cal[NUM_IMUS];

// Attitude estimators (one per IMU)
static attitude::AttitudeEstimatorQ g_estimators[NUM_IMUS];

// Console output options
static bool g_print_all_imus = true;

// Thread handles
static pthread_t g_sensor_thread;
static pthread_t g_estimator_thread;
static pthread_t g_console_thread;

// Statistics
static volatile uint32_t g_sensor_count = 0;
static volatile uint32_t g_estimator_count = 0;
static volatile uint32_t g_console_count = 0;

//=============================================================================
// Signal Handler
//=============================================================================

static void signal_handler(int signo)
{
    (void)signo;
    g_running = false;
}

//=============================================================================
// Sensor Task - Highest Priority
// Reads all IMUs at high rate and publishes to ORB
//=============================================================================

static void* sensor_task(void* arg)
{
    (void)arg;
    
    const uint32_t period_us = 1000000 / SENSOR_RATE_HZ;
    uint64_t next_time = hrt_absolute_time();
    
    while (g_running) {
        // Read all active sensors
        for (int i = 0; i < NUM_IMUS; i++) {
            if (!g_sensor_ok[i]) continue;
            
            ICM42688P::Data raw;
            if (g_sensors[i]->read(raw) != 0) continue;
            
            // Apply calibration
            calibration::Vector3f raw_accel(raw.accel[0], raw.accel[1], raw.accel[2]);
            calibration::Vector3f raw_gyro(raw.gyro[0], raw.gyro[1], raw.gyro[2]);
            
            calibration::Vector3f accel = g_accel_cal[i].correct(raw_accel);
            calibration::Vector3f gyro = g_gyro_cal[i].correct(raw_gyro);
            
            // Publish to ORB (thread-safe)
            orb::sensor_imu_s msg;
            msg.timestamp = raw.timestamp_us;
            msg.instance_id = i;
            msg.accel_m_s2[0] = accel.x;
            msg.accel_m_s2[1] = accel.y;
            msg.accel_m_s2[2] = accel.z;
            msg.gyro_rad_s[0] = gyro.x;
            msg.gyro_rad_s[1] = gyro.y;
            msg.gyro_rad_s[2] = gyro.z;
            msg.temperature_c = raw.temperature;
            
            orb::topic_sensor_imu[i].publish(msg);
        }
        
        g_sensor_count++;
        
        // Sleep until next period
        next_time += period_us;
        int64_t sleep_us = (int64_t)next_time - (int64_t)hrt_absolute_time();
        if (sleep_us > 0) {
            usleep(sleep_us);
        } else {
            // Overrun - reset timing
            next_time = hrt_absolute_time();
        }
    }
    
    return NULL;
}

//=============================================================================
// Estimator Task - Medium Priority
// Subscribes to sensor data, computes attitude, publishes result
//=============================================================================

static void* estimator_task(void* arg)
{
    (void)arg;
    
    const uint32_t period_us = 1000000 / ESTIMATOR_RATE_HZ;
    const float dt = 1.0f / ESTIMATOR_RATE_HZ;
    
    // Last update timestamps for each IMU
    uint64_t last_ts[NUM_IMUS] = {0};
    
    uint64_t next_time = hrt_absolute_time();
    
    while (g_running) {
        // Wait for calibration
        if (!g_calibrated) {
            usleep(10000);
            continue;
        }
        
        // Process each IMU
        for (int i = 0; i < NUM_IMUS; i++) {
            if (!g_sensor_ok[i]) continue;
            
            // Check for new data (poll ORB topic)
            orb::sensor_imu_s imu_data;
            if (orb::topic_sensor_imu[i].copy(imu_data)) {
                // Skip if same timestamp (no new data)
                if (imu_data.timestamp == last_ts[i]) continue;
                last_ts[i] = imu_data.timestamp;
                
                // Update attitude estimator
                g_estimators[i].update(imu_data.accel_m_s2, imu_data.gyro_rad_s, dt);
                
                // Get results
                auto euler = g_estimators[i].get_euler();
                auto quat = g_estimators[i].get_quaternion();
                auto bias = g_estimators[i].get_gyro_bias();
                
                // Publish attitude quaternion
                orb::vehicle_attitude_s att_msg;
                att_msg.timestamp = imu_data.timestamp;
                att_msg.instance_id = i;
                att_msg.q[0] = quat.w;
                att_msg.q[1] = quat.x;
                att_msg.q[2] = quat.y;
                att_msg.q[3] = quat.z;
                orb::topic_vehicle_attitude[i].publish(att_msg);
                
                // Publish Euler angles
                orb::vehicle_euler_s euler_msg;
                euler_msg.timestamp = imu_data.timestamp;
                euler_msg.instance_id = i;
                euler_msg.roll = euler.roll;
                euler_msg.pitch = euler.pitch;
                euler_msg.yaw = euler.yaw;
                euler_msg.gyro_bias[0] = bias.x;
                euler_msg.gyro_bias[1] = bias.y;
                euler_msg.gyro_bias[2] = bias.z;
                orb::topic_vehicle_euler[i].publish(euler_msg);
            }
        }
        
        g_estimator_count++;
        
        // Sleep until next period
        next_time += period_us;
        int64_t sleep_us = (int64_t)next_time - (int64_t)hrt_absolute_time();
        if (sleep_us > 0) {
            usleep(sleep_us);
        } else {
            next_time = hrt_absolute_time();
        }
    }
    
    return NULL;
}

//=============================================================================
// Console Task - Low Priority
// Subscribes to attitude and prints at low rate (console only)
//=============================================================================

static void* console_task(void* arg)
{
    (void)arg;

    const uint32_t period_us = 1000000 / CONSOLE_RATE_HZ;
    uint64_t next_time = hrt_absolute_time();

    while (g_running) {
        if (!g_calibrated) {
            usleep(100000);
            continue;
        }

        // Print normal lines, avoid "\r" tricks; keep minicom/NSH responsive.
        if (g_print_all_imus) {
            for (int i = 0; i < NUM_IMUS; i++) {
                if (!g_sensor_ok[i]) {
                    continue;
                }

                orb::vehicle_euler_s e;
                if (orb::topic_vehicle_euler[i].copy(e)) {
                    printf("[IMU%d] R:%+6.1f P:%+6.1f Y:%+6.1f  bias:%+.4f,%+.4f,%+.4f\n",
                           i,
                           (double)(e.roll * 57.2957795f),
                           (double)(e.pitch * 57.2957795f),
                           (double)(e.yaw * 57.2957795f),
                           (double)e.gyro_bias[0],
                           (double)e.gyro_bias[1],
                           (double)e.gyro_bias[2]);
                }
            }
        } else {
            // Only IMU0
            int i = 0;
            if (g_sensor_ok[i]) {
                orb::vehicle_euler_s e;
                if (orb::topic_vehicle_euler[i].copy(e)) {
                    printf("[IMU%d] R:%+6.1f P:%+6.1f Y:%+6.1f  bias:%+.4f,%+.4f,%+.4f\n",
                           i,
                           (double)(e.roll * 57.2957795f),
                           (double)(e.pitch * 57.2957795f),
                           (double)(e.yaw * 57.2957795f),
                           (double)e.gyro_bias[0],
                           (double)e.gyro_bias[1],
                           (double)e.gyro_bias[2]);
                }
            }
        }

        g_console_count++;

        next_time += period_us;
        int64_t sleep_us = (int64_t)next_time - (int64_t)hrt_absolute_time();
        if (sleep_us > 0) {
            usleep(sleep_us);
        } else {
            next_time = hrt_absolute_time();
        }
    }

    return NULL;
}

//=============================================================================
// Calibration (runs before tasks start)
//=============================================================================

static bool calibrate_sensors(void)
{
    printf("Calibrating sensors (keep UAV still)...\n");
    usleep(CALIBRATION_SETTLE_MS * 1000);
    
    for (int i = 0; i < NUM_IMUS; i++) {
        if (!g_sensor_ok[i]) continue;
        
        printf("  IMU%d: ", i);
        fflush(stdout);
        
        // Collect samples
        float accel_sum[3] = {0, 0, 0};
        float gyro_sum[3] = {0, 0, 0};
        int count = 0;
        
        for (int s = 0; s < CALIBRATION_SAMPLES; s++) {
            ICM42688P::Data data;
            if (g_sensors[i]->read(data) == 0) {
                accel_sum[0] += data.accel[0];
                accel_sum[1] += data.accel[1];
                accel_sum[2] += data.accel[2];
                gyro_sum[0] += data.gyro[0];
                gyro_sum[1] += data.gyro[1];
                gyro_sum[2] += data.gyro[2];
                count++;
            }
            usleep(2500);  // ~400 Hz
        }
        
        if (count < CALIBRATION_SAMPLES / 2) {
            printf("FAILED (too few samples)\n");
            g_sensor_ok[i] = false;
            g_active_sensors--;
            continue;
        }
        
        // Compute gyro bias
        float inv = 1.0f / count;
        calibration::Vector3f gyro_bias(
            gyro_sum[0] * inv,
            gyro_sum[1] * inv,
            gyro_sum[2] * inv
        );
        g_gyro_cal[i].set_offset(gyro_bias);
        
        // Compute accel scale from gravity
        float accel_mean[3] = {
            accel_sum[0] * inv,
            accel_sum[1] * inv,
            accel_sum[2] * inv
        };
        float g_measured = sqrtf(
            accel_mean[0]*accel_mean[0] +
            accel_mean[1]*accel_mean[1] +
            accel_mean[2]*accel_mean[2]
        );
        
        float scale = (g_measured > 1.0f) ? (9.80665f / g_measured) : 1.0f;
        calibration::Vector3f scale_vec(scale, scale, scale);
        g_accel_cal[i].set_scale(scale_vec);
        
        printf("OK (bias: %.4f,%.4f,%.4f rad/s)\n",
            (double)gyro_bias.x, (double)gyro_bias.y, (double)gyro_bias.z);
    }
    
    return g_active_sensors > 0;
}

//=============================================================================
// Initialize Sensors
//=============================================================================

static bool init_sensors(void)
{
    printf("Initializing IMU sensors...\n");
    
    for (int i = 0; i < NUM_IMUS; i++) {
        g_sensors[i] = new ICM42688P(1, g_sensor_devids[i]);
        
        if (g_sensors[i]->initialize() == 0) {
            g_sensor_ok[i] = true;
            g_active_sensors++;
            printf("  IMU%d: OK\n", i);
        } else {
            printf("  IMU%d: FAILED\n", i);
        }
    }
    
    printf("Active sensors: %d/%d\n\n", g_active_sensors, NUM_IMUS);
    return g_active_sensors > 0;
}

//=============================================================================
// Start Tasks with Proper Priorities
//=============================================================================

static bool start_tasks(void)
{
    pthread_attr_t attr;
    struct sched_param param;
    int ret;
    
    // Sensor task - highest priority
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, SENSOR_STACK_SIZE);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = SENSOR_PRIORITY;
    pthread_attr_setschedparam(&attr, &param);
    
    ret = pthread_create(&g_sensor_thread, &attr, sensor_task, NULL);
    pthread_attr_destroy(&attr);
    if (ret != 0) {
        printf("Failed to create sensor task: %d\n", ret);
        return false;
    }
    
    // Estimator task - medium priority
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, ESTIMATOR_STACK_SIZE);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = ESTIMATOR_PRIORITY;
    pthread_attr_setschedparam(&attr, &param);
    
    ret = pthread_create(&g_estimator_thread, &attr, estimator_task, NULL);
    pthread_attr_destroy(&attr);
    if (ret != 0) {
        printf("Failed to create estimator task: %d\n", ret);
        return false;
    }
    
    // Console task - low priority
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, CONSOLE_STACK_SIZE);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = CONSOLE_PRIORITY;
    pthread_attr_setschedparam(&attr, &param);
    
    ret = pthread_create(&g_console_thread, &attr, console_task, NULL);
    pthread_attr_destroy(&attr);
    if (ret != 0) {
        printf("Failed to create console task: %d\n", ret);
        return false;
    }
    
    return true;
}

//=============================================================================
// Print Usage
//=============================================================================

static void print_usage(const char* prog)
{
    printf("Usage: %s [options]\n", prog);
    printf("\n");
    printf("UAV Attitude Application - Multi-task RTOS Architecture\n");
    printf("\n");
    printf("Options:\n");
    printf("  -1        Print only IMU0 (default prints all active IMUs)\n");
    printf("  -h        Show this help\n");
    printf("\n");
    printf("Architecture:\n");
    printf("  Sensor Task     @ %d Hz (priority %d)\n", SENSOR_RATE_HZ, SENSOR_PRIORITY);
    printf("  Estimator Task  @ %d Hz (priority %d)\n", ESTIMATOR_RATE_HZ, ESTIMATOR_PRIORITY);
    printf("  Console Task    @ %d Hz (priority %d)\n", CONSOLE_RATE_HZ, CONSOLE_PRIORITY);
    printf("\n");
    printf("Data flow:\n");
    printf("  IMUs -> [ORB sensor_imu] -> Estimator -> [ORB vehicle_euler] -> console\n");
    printf("\n");
}

//=============================================================================
// Main
//=============================================================================

extern "C" int main(int argc, char* argv[])
{
    // Parse arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        if (strcmp(argv[i], "-1") == 0) {
            g_print_all_imus = false;
        }
    }
    
    // Banner (minimal console output)
    printf("\n");
    printf("========================================\n");
    printf("  UAV Attitude Application\n");
    printf("  Sensors: %d x ICM-42688-P\n", NUM_IMUS);
    printf("  Output: console only\n");
    printf("========================================\n\n");
    
    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Initialize sensors
    if (!init_sensors()) {
        printf("ERROR: No sensors available\n");
        return 1;
    }
    
    // Calibrate
    if (!calibrate_sensors()) {
        printf("ERROR: Calibration failed\n");
        return 1;
    }
    
    printf("\nStarting tasks...\n");
    
    // Start worker tasks
    if (!start_tasks()) {
        printf("ERROR: Failed to start tasks\n");
        return 1;
    }
    
    // Mark calibration complete - tasks will start processing
    g_calibrated = true;
    
    printf("Running. Press Ctrl+C to stop.\n");
    printf("Console output at %d Hz.\n\n", CONSOLE_RATE_HZ);
    
    // Main thread: periodic status (very low rate - 0.2 Hz)
    uint32_t last_sensor = 0, last_est = 0, last_console = 0;
    
    while (g_running) {
        sleep(5);  // Status every 5 seconds
        
        if (!g_running) break;
        
        // Calculate rates
        uint32_t sensor_rate = (g_sensor_count - last_sensor) / 5;
        uint32_t est_rate = (g_estimator_count - last_est) / 5;
        uint32_t console_rate = (g_console_count - last_console) / 5;
        
        last_sensor = g_sensor_count;
        last_est = g_estimator_count;
        last_console = g_console_count;
        
        printf("[Status] Sensor: %lu Hz, Estimator: %lu Hz, Console: %lu Hz\n",
            (unsigned long)sensor_rate,
            (unsigned long)est_rate,
            (unsigned long)console_rate);
    }
    
    printf("\nStopping...\n");
    
    // Stop tasks
    g_running = false;
    
    // Wait for threads
    pthread_join(g_sensor_thread, NULL);
    pthread_join(g_estimator_thread, NULL);
    pthread_join(g_console_thread, NULL);
    
    for (int i = 0; i < NUM_IMUS; i++) {
        if (g_sensors[i]) {
            delete g_sensors[i];
        }
    }
    
    printf("Done.\n");
    return 0;
}
