/****************************************************************************
 * apps/uav/estimator_app/estimator_main.cpp
 *
 * Estimator Application - Entry Point
 *
 * MỤC ĐÍCH:
 * - Ước lượng tư thế (attitude) và vị trí (position) của UAV
 * - Fusion dữ liệu từ IMU, GPS, Mag, Baro
 * - Publish kết quả cho các app khác (controller, logger)
 *
 * THIẾT KẾ:
 * - Event-driven: dùng poll() để đợi sensor data
 * - EKF2 prediction chạy mỗi IMU sample
 * - Fusion chạy khi có GPS/Mag/Baro
 * - Tách sensor selection ra module riêng
 *
 * TIMING:
 * - Trigger bởi IMU (không dùng timer riêng)
 * - Effective rate = IMU rate (250-1000 Hz)
 * - Latency từ IMU → attitude publish < 1ms
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sched.h>
#include <poll.h>
#include <errno.h>
#include <syslog.h>
#include <cmath>

#include <nuttx/clock.h>
#include <time.h>

#include <uav/uorb/uorb.hpp>
#include <uav/uorb/topics/sensor_imu.hpp>
#include <uav/uorb/topics/sensor_mag.hpp>
#include <uav/uorb/topics/sensor_baro.hpp>
#include <uav/uorb/topics/sensor_gps.hpp>
#include <uav/uorb/topics/vehicle_attitude.hpp>
#include <uav/uorb/topics/vehicle_local_position.hpp>
#include <uav/uorb/topics/ekf2_status.hpp>

static inline uint64_t uav_time_us(void)
{
    struct timespec ts;
    clock_systime_timespec(&ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/****************************************************************************
 * Cấu hình
 ****************************************************************************/

#ifndef CONFIG_UAV_ESTIMATOR_PRIORITY
#define CONFIG_UAV_ESTIMATOR_PRIORITY   240
#endif

#ifndef CONFIG_UAV_ESTIMATOR_STACKSIZE
#define CONFIG_UAV_ESTIMATOR_STACKSIZE  8192
#endif

#define PRIMARY_IMU_INSTANCE    0
#define POLL_TIMEOUT_MS         100  // Timeout cho poll (watchdog)

/****************************************************************************
 * Private Data - Static allocation
 ****************************************************************************/

// Task control
static volatile sig_atomic_t g_should_exit = 0;
static volatile bool g_is_running = false;
static pid_t g_task_pid = -1;

// Subscriptions
static int g_imu_sub = -1;
static int g_mag_sub = -1;
static int g_baro_sub = -1;
static int g_gps_sub = -1;

// Publications
static uorb::orb_advert_t g_att_pub = nullptr;
static uorb::orb_advert_t g_pos_pub = nullptr;
static uorb::orb_advert_t g_status_pub = nullptr;

// Pre-allocated message buffers
static vehicle_attitude_s g_att_msg;
static vehicle_local_position_s g_pos_msg;
static ekf2_status_s g_status_msg;

// Simple attitude state (placeholder cho EKF2)
static float g_quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float g_gyro_bias[3] = {0.0f, 0.0f, 0.0f};

// Statistics
static uint32_t g_update_count = 0;
static bool g_attitude_valid = false;

/****************************************************************************
 * Signal Handler
 ****************************************************************************/

static void signal_handler(int signo)
{
    (void)signo;
    g_should_exit = 1;
}

/****************************************************************************
 * Simple Quaternion Math (placeholder - sẽ thay bằng EKF2)
 ****************************************************************************/

static void quaternion_normalize(float q[4])
{
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 0.0f) {
        float inv_norm = 1.0f / norm;
        q[0] *= inv_norm;
        q[1] *= inv_norm;
        q[2] *= inv_norm;
        q[3] *= inv_norm;
    }
}

static void quaternion_integrate(float q[4], const float gyro[3], float dt)
{
    // First-order integration: q = q + 0.5 * q * omega * dt
    float dq[4];
    dq[0] = 0.5f * (-q[1]*gyro[0] - q[2]*gyro[1] - q[3]*gyro[2]) * dt;
    dq[1] = 0.5f * ( q[0]*gyro[0] + q[2]*gyro[2] - q[3]*gyro[1]) * dt;
    dq[2] = 0.5f * ( q[0]*gyro[1] - q[1]*gyro[2] + q[3]*gyro[0]) * dt;
    dq[3] = 0.5f * ( q[0]*gyro[2] + q[1]*gyro[1] - q[2]*gyro[0]) * dt;

    q[0] += dq[0];
    q[1] += dq[1];
    q[2] += dq[2];
    q[3] += dq[3];

    quaternion_normalize(q);
}

static void quaternion_to_euler(const float q[4], float* roll, float* pitch, float* yaw)
{
    // ZYX Euler angles from quaternion
    *roll  = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2]));
    *pitch = asinf(2.0f*(q[0]*q[2] - q[3]*q[1]));
    *yaw   = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]));
}

/****************************************************************************
 * process_imu - Xử lý IMU và update attitude
 *
 * Placeholder cho EKF2 prediction step
 ****************************************************************************/

static uint64_t g_last_imu_time = 0;

static void process_imu(const sensor_imu_s& imu)
{
    // Tính dt
    float dt = 0.001f;  // Default 1ms
    if (g_last_imu_time > 0 && imu.timestamp_us > g_last_imu_time) {
        dt = (imu.timestamp_us - g_last_imu_time) * 1e-6f;
        if (dt > 0.1f) dt = 0.1f;  // Limit
        if (dt < 0.0001f) dt = 0.0001f;
    }
    g_last_imu_time = imu.timestamp_us;

    // Bù bias
    float gyro_corrected[3];
    gyro_corrected[0] = imu.gyro[0] - g_gyro_bias[0];
    gyro_corrected[1] = imu.gyro[1] - g_gyro_bias[1];
    gyro_corrected[2] = imu.gyro[2] - g_gyro_bias[2];

    // Integrate quaternion
    quaternion_integrate(g_quaternion, gyro_corrected, dt);

    g_attitude_valid = true;
}

/****************************************************************************
 * publish_attitude - Publish attitude message
 ****************************************************************************/

static void publish_attitude(uint64_t timestamp)
{
    g_att_msg.timestamp_us = timestamp;

    // Quaternion
    g_att_msg.q[0] = g_quaternion[0];
    g_att_msg.q[1] = g_quaternion[1];
    g_att_msg.q[2] = g_quaternion[2];
    g_att_msg.q[3] = g_quaternion[3];

    // Euler angles
    quaternion_to_euler(g_quaternion, &g_att_msg.roll, &g_att_msg.pitch, &g_att_msg.yaw);

    // Angular rates (placeholder)
    g_att_msg.rollspeed = 0;
    g_att_msg.pitchspeed = 0;
    g_att_msg.yawspeed = 0;

    g_att_msg.instance = 0;

    if (g_att_pub) {
        uorb::orb_publish(ORB_ID(vehicle_attitude), g_att_pub, &g_att_msg);
    }
}

/****************************************************************************
 * publish_status - Publish EKF2 status
 ****************************************************************************/

static void publish_status(uint64_t timestamp)
{
    g_status_msg.timestamp_us = timestamp;

    g_status_msg.gyro_bias[0] = g_gyro_bias[0];
    g_status_msg.gyro_bias[1] = g_gyro_bias[1];
    g_status_msg.gyro_bias[2] = g_gyro_bias[2];

    g_status_msg.tilt_align = g_attitude_valid;
    g_status_msg.yaw_align = false;  // Cần mag
    g_status_msg.gps_fused = false;
    g_status_msg.baro_fused = false;
    g_status_msg.mag_fused = false;

    if (g_status_pub) {
        uorb::orb_publish(ORB_ID(ekf2_status), g_status_pub, &g_status_msg);
    }
}

/****************************************************************************
 * estimator_thread_main - Main loop
 ****************************************************************************/

static int estimator_thread_main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    //=========================================================================
    // PHASE 1: Set priority
    //=========================================================================

    struct sched_param param;
    param.sched_priority = CONFIG_UAV_ESTIMATOR_PRIORITY;
    sched_setscheduler(0, SCHED_FIFO, &param);

    //=========================================================================
    // PHASE 2: Subscribe to sensor topics
    //=========================================================================

    syslog(LOG_INFO, "[estimator] Subscribing to sensors...\n");

    g_imu_sub = uorb::orb_subscribe_multi(ORB_ID(sensor_imu), PRIMARY_IMU_INSTANCE);
    if (g_imu_sub < 0) {
        syslog(LOG_ERR, "[estimator] Failed to subscribe to IMU\n");
        return -1;
    }

    // Optional sensors (may not exist yet)
    g_mag_sub = uorb::orb_subscribe(ORB_ID(sensor_mag));
    g_baro_sub = uorb::orb_subscribe(ORB_ID(sensor_baro));
    g_gps_sub = uorb::orb_subscribe(ORB_ID(sensor_gps));

    //=========================================================================
    // PHASE 3: Advertise output topics
    //=========================================================================

    memset(&g_att_msg, 0, sizeof(g_att_msg));
    memset(&g_pos_msg, 0, sizeof(g_pos_msg));
    memset(&g_status_msg, 0, sizeof(g_status_msg));

    g_att_pub = uorb::orb_advertise(ORB_ID(vehicle_attitude), &g_att_msg);
    g_pos_pub = uorb::orb_advertise(ORB_ID(vehicle_local_position), &g_pos_msg);
    g_status_pub = uorb::orb_advertise(ORB_ID(ekf2_status), &g_status_msg);

    //=========================================================================
    // PHASE 4: Init state
    //=========================================================================

    g_quaternion[0] = 1.0f;
    g_quaternion[1] = 0.0f;
    g_quaternion[2] = 0.0f;
    g_quaternion[3] = 0.0f;

    g_is_running = true;
    syslog(LOG_INFO, "[estimator] Waiting for IMU data...\n");

    //=========================================================================
    // PHASE 5: Main loop - orb_check based (uORB không hỗ trợ poll)
    //=========================================================================

    /* Không dùng poll() vì uORB fd không phải real file descriptor.
     * Thay vào đó dùng orb_check() với usleep() */

    bool first_imu_received = false;

    while (!g_should_exit) {
        /* Check for IMU data */
        bool imu_updated = false;
        if (uorb::orb_check(g_imu_sub, &imu_updated) == 0 && imu_updated) {
            sensor_imu_s imu;
            if (uorb::orb_copy(ORB_ID(sensor_imu), g_imu_sub, &imu) == 0) {
                if (!first_imu_received) {
                    syslog(LOG_INFO, "[estimator] First IMU sample received\n");
                    first_imu_received = true;
                }

                process_imu(imu);
                g_update_count++;

                uint64_t now = uav_time_us();

                /* Publish attitude every update */
                if (g_attitude_valid) {
                    publish_attitude(now);
                }

                /* Publish status less frequently */
                if (g_update_count % 25 == 0) {
                    publish_status(now);
                }
            }
        } else {
            /* Không có data mới, sleep 1ms để không busy-loop quá nặng */
            usleep(1000);
        }
    }

    //=========================================================================
    // PHASE 6: Cleanup
    //=========================================================================

    g_is_running = false;

    uorb::orb_unsubscribe(g_imu_sub);
    uorb::orb_unsubscribe(g_mag_sub);
    uorb::orb_unsubscribe(g_baro_sub);
    uorb::orb_unsubscribe(g_gps_sub);

    if (g_att_pub) uorb::orb_unadvertise(g_att_pub);
    if (g_pos_pub) uorb::orb_unadvertise(g_pos_pub);
    if (g_status_pub) uorb::orb_unadvertise(g_status_pub);

    syslog(LOG_INFO, "[estimator] Stopped after %lu updates\n",
           (unsigned long)g_update_count);

    return 0;
}

/****************************************************************************
 * print_usage
 ****************************************************************************/

static void print_usage(void)
{
    printf("Usage: estimator <command>\n");
    printf("\nCommands:\n");
    printf("  start     Khởi động estimator task\n");
    printf("  stop      Dừng estimator task\n");
    printf("  status    Xem trạng thái\n");
}

/****************************************************************************
 * estimator_main - Entry point
 ****************************************************************************/

extern "C" int estimator_main(int argc, char *argv[])
{
    if (argc < 2) {
        print_usage();
        return 1;
    }

    const char* cmd = argv[1];

    //-------------------------------------------------------------------------
    // Command: start
    //-------------------------------------------------------------------------

    if (strcmp(cmd, "start") == 0) {
        if (g_is_running) {
            printf("[estimator] Already running\n");
            return 0;
        }

        g_should_exit = 0;
        g_update_count = 0;
        g_attitude_valid = false;

        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);

        g_task_pid = task_create(
            "estimator",
            CONFIG_UAV_ESTIMATOR_PRIORITY,
            CONFIG_UAV_ESTIMATOR_STACKSIZE,
            estimator_thread_main,
            nullptr
        );

        if (g_task_pid < 0) {
            printf("[estimator] Failed to create task: %d\n", errno);
            return -errno;
        }

        printf("[estimator] Started (pid=%d)\n", g_task_pid);
        return 0;
    }

    //-------------------------------------------------------------------------
    // Command: stop
    //-------------------------------------------------------------------------

    if (strcmp(cmd, "stop") == 0) {
        if (!g_is_running) {
            printf("[estimator] Not running\n");
            return 0;
        }

        g_should_exit = 1;

        for (int i = 0; i < 20 && g_is_running; i++) {
            usleep(100000);
        }

        if (g_is_running) {
            printf("[estimator] Timeout waiting for task\n");
            return 1;
        }

        printf("[estimator] Stopped\n");
        return 0;
    }

    //-------------------------------------------------------------------------
    // Command: status
    //-------------------------------------------------------------------------

    if (strcmp(cmd, "status") == 0) {
        if (!g_is_running) {
            printf("[estimator] Not running\n");
            return 0;
        }

        printf("[estimator] Running\n");
        printf("  Update count:    %lu\n", (unsigned long)g_update_count);
        printf("  Attitude valid:  %s\n", g_attitude_valid ? "YES" : "NO");
        printf("  Roll:  %+7.2f deg\n", (double)(g_att_msg.roll * 57.2957795f));
        printf("  Pitch: %+7.2f deg\n", (double)(g_att_msg.pitch * 57.2957795f));
        printf("  Yaw:   %+7.2f deg\n", (double)(g_att_msg.yaw * 57.2957795f));

        return 0;
    }

    print_usage();
    return 1;
}
