/****************************************************************************
 * apps/examples/uav_states_v1/main.cpp
 *
 * UAV Attitude Estimation - Main Application (Central Coordinator)
 *
 * ARCHITECTURE OVERVIEW:
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                         Main Application                                │
 * │                    (Coordinator / Dispatcher)                           │
 * └─────────────────────────────────────────────────────────────────────────┘
 *          │                      │                        │
 *          ▼                      ▼                        ▼
 *    ┌──────────┐          ┌───────────────┐        ┌─────────────┐
 *    │ Sensors  │ ──────►  │  Estimator    │ ─────► │  Display    │
 *    │ Module   │ sensor_  │  Module       │  att_  │  (syslog)   │
 *    │ (Thread) │ imu      │  (called)     │        │             │
 *    └──────────┘          └───────────────┘        └─────────────┘
 *         │                       │
 *         │                       │
 *    ┌────▼───────────────────────▼────┐
 *    │         uORB Topics            │
 *    │  sensor_imu[4]  vehicle_att[4] │
 *    └─────────────────────────────────┘
 *
 * DATA FLOW (Pub/Sub):
 * 1. ImuModule runs in separate thread @ 100Hz, reads sensors.
 * 2. ImuModule publishes sensor_imu_s to topics.
 * 3. Main loop calls AttitudeModule.update() periodically.
 * 4. AttitudeModule subscribes to sensor_imu, computes attitude.
 * 5. AttitudeModule publishes vehicle_attitude_s.
 * 6. Main loop reads attitude and displays via syslog.
 *
 * WHY THIS DESIGN:
 * - Sensor I/O isolated in one thread -> no SPI bus conflicts.
 * - Estimator runs in main thread -> easy debugging.
 * - syslog for display -> non-blocking, no console lag.
 * - Each module is independent -> easy to add new modules.
 *
 * FUTURE EXPANSION:
 * - Add MagModule for BMM150 magnetometer.
 * - Add BaroModule for MS5611 barometer.
 * - Add LoggerModule for SD card logging.
 * - Add EKF2 for full state estimation.
 * - Add ControllerModule for attitude control.
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <syslog.h>
#include <cmath>

#include "uorb/uorb.hpp"
#include "uorb/topics.hpp"
#include "modules/sensors/imu_module.hpp"
#include "modules/estimator/attitude_module.hpp"
#include "platforms/nuttx/hrt/hrt.h"

/****************************************************************************
 * Configuration
 ****************************************************************************/

#define MAIN_LOOP_RATE_HZ   100   // Main loop frequency
#define DISPLAY_RATE_HZ     10    // Attitude display frequency

static constexpr float RAD2DEG = 57.2957795f;

/****************************************************************************
 * Global State
 ****************************************************************************/

static volatile bool g_running = true;
static modules::sensors::ImuModule g_imu_module;
static modules::estimator::AttitudeModule g_attitude_module;

/****************************************************************************
 * Signal Handler - Clean shutdown on Ctrl+C
 ****************************************************************************/

static void signal_handler(int signo)
{
    (void)signo;
    g_running = false;
    g_imu_module.request_stop();
}

/****************************************************************************
 * Display Functions - Uses syslog for non-blocking output
 *
 * syslog is asynchronous and doesn't block the calling thread,
 * making it ideal for real-time display without affecting timing.
 ****************************************************************************/

static void print_header(int num_imus)
{
    printf("\n");
    printf("========================================\n");
    printf("  UAV Attitude Estimation\n");
    printf("  ICM-42688-P x%d @ %d Hz\n", num_imus, MAIN_LOOP_RATE_HZ);
    printf("========================================\n\n");

    printf("%-8s", "Time");
    for (int i = 0; i < num_imus; i++) {
        printf("  IMU%d Roll   Pitch    Yaw ", i);
    }
    printf("\n");

    printf("--------");
    for (int i = 0; i < num_imus; i++) {
        printf("  ---------------------------");
    }
    printf("\n");
}

/**
 * @brief Display attitude using syslog (non-blocking)
 *
 * We use syslog instead of printf because:
 * 1. syslog buffers messages and writes asynchronously.
 * 2. printf flushes to serial and can block for ~10ms.
 * 3. Blocking affects loop timing and sensor reading.
 */
static void display_attitude(uint32_t elapsed_sec, int display_count)
{
    // Build output string first
    char buf[256];
    int pos = 0;

    pos += snprintf(buf + pos, sizeof(buf) - pos, "%5lu.%01lus",
                    (unsigned long)elapsed_sec,
                    (unsigned long)(display_count % 10));

    for (int i = 0; i < g_attitude_module.num_estimators(); i++) {
        uorb::vehicle_attitude_s att;
        if (g_attitude_module.get_attitude(i, att)) {
            pos += snprintf(buf + pos, sizeof(buf) - pos, "  %+7.1f %+7.1f %+7.1f",
                           (double)(att.roll * RAD2DEG),
                           (double)(att.pitch * RAD2DEG),
                           (double)(att.yaw * RAD2DEG));
        } else {
            pos += snprintf(buf + pos, sizeof(buf) - pos, "  %7s %7s %7s", "-", "-", "-");
        }
    }

    // Use syslog for async output (LOG_INFO goes to console)
    syslog(LOG_INFO, "%s\n", buf);
}

/****************************************************************************
 * Main Application Entry Point
 *
 * Responsibilities:
 * 1. Initialize modules.
 * 2. Start sensor thread.
 * 3. Run main loop (call estimator, display).
 * 4. Handle shutdown.
 ****************************************************************************/

extern "C" int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    //=========================================================================
    // PHASE 1: Initialize Sensor Module
    //=========================================================================

    int num_imus = g_imu_module.init();
    if (num_imus == 0) {
        syslog(LOG_ERR, "[main] ERROR: No IMU sensors available!\n");
        printf("ERROR: No IMU sensors available!\n");
        return 1;
    }

    //=========================================================================
    // PHASE 2: Calibrate Sensors
    //=========================================================================

    if (!g_imu_module.calibrate()) {
        syslog(LOG_ERR, "[main] ERROR: Calibration failed!\n");
        printf("ERROR: Calibration failed!\n");
        return 1;
    }

    //=========================================================================
    // PHASE 3: Initialize Estimator Module
    //
    // Pass IMU topics from sensor module to estimator module.
    // This creates the pub/sub connection.
    //=========================================================================

    // Get pointers to IMU topics (one per sensor)
    uorb::Topic<uorb::sensor_imu_s, 8>* imu_topics[modules::sensors::MAX_IMUS];
    for (int i = 0; i < modules::sensors::MAX_IMUS; i++) {
        if (g_imu_module.is_active(i)) {
            imu_topics[i] = &g_imu_module.get_topic(i);
        } else {
            imu_topics[i] = nullptr;
        }
    }

    // Initialize estimator with IMU topics
    g_attitude_module.init(imu_topics[0], num_imus);

    //=========================================================================
    // PHASE 4: Start Sensor Thread
    //
    // Sensor thread runs independently, publishing to topics.
    // Main thread consumes topics via estimator.
    //=========================================================================

    if (g_imu_module.start() != 0) {
        syslog(LOG_ERR, "[main] ERROR: Failed to start sensor thread!\n");
        printf("ERROR: Failed to start sensor thread!\n");
        return 1;
    }

    //=========================================================================
    // PHASE 5: Main Loop
    //
    // - Runs at MAIN_LOOP_RATE_HZ.
    // - Calls estimator.update() each cycle.
    // - Displays attitude at DISPLAY_RATE_HZ.
    //=========================================================================

    print_header(num_imus);
    syslog(LOG_INFO, "[main] Running at %d Hz. Press Ctrl+C to stop.\n", MAIN_LOOP_RATE_HZ);

    const uint32_t loop_period_us = 1000000 / MAIN_LOOP_RATE_HZ;
    const int display_divider = MAIN_LOOP_RATE_HZ / DISPLAY_RATE_HZ;

    uint64_t next_time = hrt_absolute_time();
    uint64_t start_time = next_time;
    uint32_t loop_count = 0;

    while (g_running) {
        //---------------------------------------------------------------------
        // Update estimator (consumes IMU topics, produces attitude)
        //---------------------------------------------------------------------
        g_attitude_module.update();

        //---------------------------------------------------------------------
        // Display at lower rate
        //---------------------------------------------------------------------
        loop_count++;
        if (loop_count % display_divider == 0) {
            uint32_t elapsed = (hrt_absolute_time() - start_time) / 1000000;
            display_attitude(elapsed, loop_count / display_divider);
        }

        //---------------------------------------------------------------------
        // Sleep until next cycle
        //---------------------------------------------------------------------
        next_time += loop_period_us;
        int64_t sleep_us = (int64_t)next_time - (int64_t)hrt_absolute_time();
        if (sleep_us > 0) {
            usleep(sleep_us);
        } else {
            // Overrun - reset timing
            next_time = hrt_absolute_time();
        }
    }

    //=========================================================================
    // PHASE 6: Shutdown
    //=========================================================================

    g_imu_module.stop();
    syslog(LOG_INFO, "[main] Stopped.\n");
    printf("\nStopped.\n");

    return 0;
}
