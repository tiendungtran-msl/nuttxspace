/****************************************************************************
 * apps/examples/uav_states_v1/uav_attitude_app.cpp
 *
 * Simple UAV Attitude Application - Console Output Only
 * 
 * Reads 4x ICM-42688-P IMUs, estimates attitude, prints to console.
 * No external telemetry, no Python tool connection.
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <errno.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#include "drivers/imu/icm42688p/icm42688p.hpp"
#include "calibration/sensor_calibration.hpp"
#include "platforms/boards/spi_config.h"
#include "platforms/nuttx/hrt/hrt.h"

using namespace drivers::imu;

//=============================================================================
// Configuration
//=============================================================================

#define NUM_IMUS            4
#define SAMPLE_RATE_HZ      100
#define PRINT_RATE_HZ       5      // Print every 200ms
#define CALIBRATION_SAMPLES 200    // ~2 seconds at 100Hz

//=============================================================================
// Simple Complementary Filter for Attitude
//=============================================================================

struct AttitudeState {
    float roll;     // radians
    float pitch;    // radians
    float yaw;      // radians
    bool valid;
};

static void attitude_init(AttitudeState* att) {
    att->roll = 0.0f;
    att->pitch = 0.0f;
    att->yaw = 0.0f;
    att->valid = false;
}

static void attitude_update(AttitudeState* att, 
                           const float accel[3], 
                           const float gyro[3], 
                           float dt)
{
    const float alpha = 0.98f;  // Complementary filter coefficient
    
    // Accelerometer-based angles (only roll/pitch)
    float accel_roll = atan2f(accel[1], accel[2]);
    float accel_pitch = atan2f(-accel[0], sqrtf(accel[1]*accel[1] + accel[2]*accel[2]));
    
    if (!att->valid) {
        // First update - initialize from accelerometer
        att->roll = accel_roll;
        att->pitch = accel_pitch;
        att->yaw = 0.0f;
        att->valid = true;
    } else {
        // Complementary filter
        att->roll = alpha * (att->roll + gyro[0] * dt) + (1.0f - alpha) * accel_roll;
        att->pitch = alpha * (att->pitch + gyro[1] * dt) + (1.0f - alpha) * accel_pitch;
        att->yaw += gyro[2] * dt;  // Gyro only for yaw (no magnetometer)
        
        // Normalize yaw to [-180, 180]
        while (att->yaw > M_PI) att->yaw -= 2.0f * M_PI;
        while (att->yaw < -M_PI) att->yaw += 2.0f * M_PI;
    }
}

//=============================================================================
// Global State
//=============================================================================

static volatile bool g_running = true;

static ICM42688P* g_sensors[NUM_IMUS] = {nullptr};
static bool g_sensor_ok[NUM_IMUS] = {false};
static int g_active_sensors = 0;

static const uint32_t g_sensor_devids[NUM_IMUS] = {
    SPIDEV_IMU, SPIDEV_IMU1, SPIDEV_IMU2, SPIDEV_IMU3
};

// Calibration offsets
static float g_gyro_bias[NUM_IMUS][3];
static float g_accel_scale[NUM_IMUS];

// Attitude states
static AttitudeState g_attitude[NUM_IMUS];

//=============================================================================
// Signal Handler
//=============================================================================

static void signal_handler(int signo)
{
    (void)signo;
    g_running = false;
}

//=============================================================================
// Initialize Sensors
//=============================================================================

static bool init_sensors(void)
{
    printf("Initializing IMU sensors...\n");
    
    for (int i = 0; i < NUM_IMUS; i++) {
        printf("  [IMU%d] ", i);
        fflush(stdout);
        
        g_sensors[i] = new ICM42688P(1, g_sensor_devids[i]);
        
        int ret = g_sensors[i]->initialize();
        if (ret == 0) {
            g_sensor_ok[i] = true;
            g_active_sensors++;
            printf("OK\n");
        } else {
            printf("FAILED (error %d)\n", ret);
        }
    }
    
    printf("\nActive sensors: %d/%d\n", g_active_sensors, NUM_IMUS);
    return g_active_sensors > 0;
}

//=============================================================================
// Calibrate Sensors
//=============================================================================

static bool calibrate_sensors(void)
{
    printf("\nCalibrating... (keep UAV still)\n");
    usleep(500000);  // Wait 500ms to settle
    
    for (int i = 0; i < NUM_IMUS; i++) {
        if (!g_sensor_ok[i]) continue;
        
        printf("  [IMU%d] ", i);
        fflush(stdout);
        
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
            usleep(10000);  // 100Hz
        }
        
        if (count < CALIBRATION_SAMPLES / 2) {
            printf("FAILED (insufficient samples)\n");
            g_sensor_ok[i] = false;
            g_active_sensors--;
            continue;
        }
        
        float inv = 1.0f / count;
        
        // Gyro bias
        g_gyro_bias[i][0] = gyro_sum[0] * inv;
        g_gyro_bias[i][1] = gyro_sum[1] * inv;
        g_gyro_bias[i][2] = gyro_sum[2] * inv;
        
        // Accel scale (normalize to 9.81 m/s²)
        float ax = accel_sum[0] * inv;
        float ay = accel_sum[1] * inv;
        float az = accel_sum[2] * inv;
        float g_measured = sqrtf(ax*ax + ay*ay + az*az);
        g_accel_scale[i] = (g_measured > 1.0f) ? (9.80665f / g_measured) : 1.0f;
        
        // Initialize attitude
        attitude_init(&g_attitude[i]);
        
        printf("OK (gyro bias: %.3f, %.3f, %.3f rad/s)\n",
               (double)g_gyro_bias[i][0],
               (double)g_gyro_bias[i][1],
               (double)g_gyro_bias[i][2]);
    }
    
    return g_active_sensors > 0;
}

//=============================================================================
// Convert radians to degrees
//=============================================================================

static inline float rad2deg(float rad) {
    return rad * 57.2957795f;
}

//=============================================================================
// Main
//=============================================================================

extern "C" int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;
    
    printf("\n");
    printf("========================================\n");
    printf("  UAV Attitude - Console Mode\n");
    printf("  %d x ICM-42688-P IMU Sensors\n", NUM_IMUS);
    printf("========================================\n\n");
    
    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Initialize sensors
    if (!init_sensors()) {
        printf("ERROR: No sensors available!\n");
        return 1;
    }
    
    // Calibrate
    if (!calibrate_sensors()) {
        printf("ERROR: Calibration failed!\n");
        return 1;
    }
    
    printf("\nRunning at %d Hz. Press Ctrl+C to stop.\n\n", SAMPLE_RATE_HZ);
    
    // Print header
    printf("%-8s", "Time");
    for (int i = 0; i < NUM_IMUS; i++) {
        if (g_sensor_ok[i]) {
            printf("  IMU%d Roll   Pitch   Yaw  ", i);
        }
    }
    printf("\n");
    
    // Divider
    printf("--------");
    for (int i = 0; i < NUM_IMUS; i++) {
        if (g_sensor_ok[i]) {
            printf("  -------------------------");
        }
    }
    printf("\n");
    
    // Main loop
    const uint32_t sample_period_us = 1000000 / SAMPLE_RATE_HZ;
    const int print_divider = SAMPLE_RATE_HZ / PRINT_RATE_HZ;
    const float dt = 1.0f / SAMPLE_RATE_HZ;
    
    uint64_t next_time = hrt_absolute_time();
    uint32_t loop_count = 0;
    uint32_t start_time = hrt_absolute_time() / 1000000;
    
    while (g_running) {
        // Read and update all sensors
        for (int i = 0; i < NUM_IMUS; i++) {
            if (!g_sensor_ok[i]) continue;
            
            ICM42688P::Data raw;
            if (g_sensors[i]->read(raw) != 0) continue;
            
            // Apply calibration
            float accel[3] = {
                raw.accel[0] * g_accel_scale[i],
                raw.accel[1] * g_accel_scale[i],
                raw.accel[2] * g_accel_scale[i]
            };
            float gyro[3] = {
                raw.gyro[0] - g_gyro_bias[i][0],
                raw.gyro[1] - g_gyro_bias[i][1],
                raw.gyro[2] - g_gyro_bias[i][2]
            };
            
            // Update attitude
            attitude_update(&g_attitude[i], accel, gyro, dt);
        }
        
        // Print at lower rate
        if (++loop_count % print_divider == 0) {
            uint32_t elapsed = (hrt_absolute_time() / 1000000) - start_time;
            
            printf("%5lu.%01lus", 
                   (unsigned long)(elapsed), 
                   (unsigned long)((loop_count / print_divider) % 10));
            
            for (int i = 0; i < NUM_IMUS; i++) {
                if (!g_sensor_ok[i]) continue;
                
                if (g_attitude[i].valid) {
                    printf("  %+7.1f %+7.1f %+7.1f",
                           (double)rad2deg(g_attitude[i].roll),
                           (double)rad2deg(g_attitude[i].pitch),
                           (double)rad2deg(g_attitude[i].yaw));
                } else {
                    printf("      ---     ---     ---");
                }
            }
            printf("\n");
        }
        
        // Sleep until next sample
        next_time += sample_period_us;
        int64_t sleep_us = (int64_t)next_time - (int64_t)hrt_absolute_time();
        if (sleep_us > 0) {
            usleep(sleep_us);
        } else {
            // Overrun - reset timing
            next_time = hrt_absolute_time();
        }
    }
    
    printf("\nStopping...\n");
    
    // Cleanup
    for (int i = 0; i < NUM_IMUS; i++) {
        if (g_sensors[i]) {
            delete g_sensors[i];
            g_sensors[i] = nullptr;
        }
    }
    
    printf("Done.\n");
    return 0;
}
