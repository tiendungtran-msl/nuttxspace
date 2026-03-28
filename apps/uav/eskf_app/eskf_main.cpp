/****************************************************************************
 * apps/uav/eskf_app/eskf_main.cpp
 *
 * ESKF Estimator Application - Entry Point
 *
 * MỤC ĐÍCH:
 * - Ước lượng tư thế (attitude) bằng Error-State Kalman Filter
 * - Sử dụng chủ yếu dữ liệu IMU (không cần GPS)
 * - Ma trận 9×9 (thay vì 15×15 khi có GPS)
 *
 * THUẬT TOÁN:
 * - ESKF 9-state: δθ(3), δbg(3), δba(3)
 * - Prediction: gyroscope propagation
 * - Update: accelerometer gravity observation
 * - Dựa trên J. Solà "Quaternion kinematics for the ESKF"
 *
 * LỆNH:
 *   eskf start    - Khởi động ESKF estimator
 *   eskf stop     - Dừng estimator
 *   eskf status   - Xem trạng thái và tư thế hiện tại
 *
 * TIMING:
 * - Trigger bởi IMU data (~1000 Hz)
 * - Predict + Update mỗi mẫu IMU
 * - Publish attitude @ mỗi update
 * - Publish status @ 25 Hz
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sched.h>
#include <errno.h>
#include <syslog.h>
#include <cmath>

#include <nuttx/clock.h>
#include <time.h>

#include <uav/uorb/uorb.hpp>
#include <uav/uorb/topics/sensor_imu.hpp>
#include <uav/uorb/topics/vehicle_attitude.hpp>
#include <uav/uorb/topics/ekf2_status.hpp>
#include <uav/lib/eskf/eskf.hpp>

static constexpr float RAD_TO_DEG = 57.295779513082323f;

/****************************************************************************
 * Hàm lấy thời gian hệ thống (microseconds)
 ****************************************************************************/

static inline uint64_t eskf_time_us(void)
{
    struct timespec ts;
    clock_systime_timespec(&ts);
    return (uint64_t)ts.tv_sec * 1000000ULL
         + (uint64_t)ts.tv_nsec / 1000ULL;
}

/****************************************************************************
 * Cấu hình
 ****************************************************************************/

#ifndef CONFIG_UAV_ESKF_PRIORITY
#define CONFIG_UAV_ESKF_PRIORITY    240
#endif

#ifndef CONFIG_UAV_ESKF_STACKSIZE
#define CONFIG_UAV_ESKF_STACKSIZE   8192
#endif

#define PRIMARY_IMU_INSTANCE    0

/****************************************************************************
 * Private Data - Static allocation (không dùng heap)
 ****************************************************************************/

/* Task control */
static volatile sig_atomic_t g_should_exit = 0;
static volatile bool         g_is_running  = false;
static pid_t                 g_task_pid    = -1;

/* uORB handles */
static int                   g_imu_sub    = -1;
static uorb::orb_advert_t   g_att_pub    = nullptr;
static uorb::orb_advert_t   g_status_pub = nullptr;

/* Pre-allocated message buffers */
static vehicle_attitude_s    g_att_msg;
static ekf2_status_s         g_status_msg;

/* ESKF filter instance */
static eskf::Eskf            g_eskf;
static eskf::EskfOutput      g_output;

/* Statistics */
static uint32_t g_update_count   = 0;
static uint32_t g_predict_count  = 0;
static uint32_t g_update_skip    = 0;

/****************************************************************************
 * Signal Handler
 ****************************************************************************/

static void signal_handler(int signo)
{
    (void)signo;
    g_should_exit = 1;
}

/****************************************************************************
 * publish_attitude - Publish ước lượng tư thế qua uORB
 ****************************************************************************/

static void publish_attitude(uint64_t timestamp)
{
    g_att_msg.timestamp_us = timestamp;

    g_att_msg.q[0] = g_output.q[0];
    g_att_msg.q[1] = g_output.q[1];
    g_att_msg.q[2] = g_output.q[2];
    g_att_msg.q[3] = g_output.q[3];

    g_att_msg.roll  = g_output.roll;
    g_att_msg.pitch = g_output.pitch;
    g_att_msg.yaw   = g_output.yaw;

    g_att_msg.rollspeed  = 0.0f;
    g_att_msg.pitchspeed = 0.0f;
    g_att_msg.yawspeed   = 0.0f;
    g_att_msg.instance   = 0;

    if (g_att_pub) {
        uorb::orb_publish(ORB_ID(vehicle_attitude), g_att_pub, &g_att_msg);
    }
}

/****************************************************************************
 * publish_status - Publish trạng thái ESKF qua uORB
 ****************************************************************************/

static void publish_status(uint64_t timestamp)
{
    g_status_msg.timestamp_us = timestamp;

    for (int i = 0; i < 3; i++) {
        g_status_msg.gyro_bias[i]  = g_output.gyro_bias[i];
        g_status_msg.accel_bias[i] = g_output.accel_bias[i];
    }

    g_status_msg.tilt_align = g_output.valid;
    g_status_msg.yaw_align  = false;   /* Yaw không quan sát được */
    g_status_msg.gps_fused  = false;   /* Không dùng GPS */
    g_status_msg.baro_fused = false;
    g_status_msg.mag_fused  = false;

    /* Covariance diagonals */
    for (int i = 0; i < 3; i++) {
        g_status_msg.pos_var[i] = 0.0f;
        g_status_msg.vel_var[i] = 0.0f;
    }

    if (g_status_pub) {
        uorb::orb_publish(ORB_ID(ekf2_status), g_status_pub, &g_status_msg);
    }
}

/****************************************************************************
 * eskf_thread_main - Main loop
 ****************************************************************************/

static int eskf_thread_main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    /*=====================================================================
     * PHASE 1: Set FIFO priority
     *====================================================================*/
    struct sched_param param;
    param.sched_priority = CONFIG_UAV_ESKF_PRIORITY;
    sched_setscheduler(0, SCHED_FIFO, &param);

    /*=====================================================================
     * PHASE 2: Subscribe to IMU topic
     *====================================================================*/
    syslog(LOG_INFO, "[eskf] Subscribing to IMU...\n");

    g_imu_sub = uorb::orb_subscribe_multi(ORB_ID(sensor_imu),
                                            PRIMARY_IMU_INSTANCE);
    if (g_imu_sub < 0) {
        syslog(LOG_ERR, "[eskf] Failed to subscribe to IMU\n");
        return -1;
    }

    /*=====================================================================
     * PHASE 3: Advertise output topics
     *====================================================================*/
    memset(&g_att_msg,    0, sizeof(g_att_msg));
    memset(&g_status_msg, 0, sizeof(g_status_msg));

    g_att_pub    = uorb::orb_advertise(ORB_ID(vehicle_attitude), &g_att_msg);
    g_status_pub = uorb::orb_advertise(ORB_ID(ekf2_status), &g_status_msg);

    /*=====================================================================
     * PHASE 4: Initialize ESKF
     *====================================================================*/
    eskf::EskfConfig config = eskf::eskf_default_config();
    g_eskf.init(config);

    g_is_running = true;
    syslog(LOG_INFO, "[eskf] ESKF 9-state initialized, waiting for IMU...\n");
    syslog(LOG_INFO, "[eskf] State: [dtheta(3), dbg(3), dba(3)] = 9-dim\n");
    syslog(LOG_INFO, "[eskf] Predict: gyro | Update: accel gravity\n");

    /*=====================================================================
     * PHASE 5: Main loop
     *====================================================================*/
    bool first_imu = false;

    while (!g_should_exit) {
        bool imu_updated = false;

        if (uorb::orb_check(g_imu_sub, &imu_updated) == 0 && imu_updated) {
            sensor_imu_s imu;
            if (uorb::orb_copy(ORB_ID(sensor_imu), g_imu_sub, &imu) == 0) {

                if (!first_imu) {
                    syslog(LOG_INFO, "[eskf] First IMU sample received\n");
                    first_imu = true;
                }

                /* ESKF: predict + update */
                bool valid = g_eskf.process_imu(imu.accel, imu.gyro,
                                                imu.timestamp_us);

                if (valid && g_eskf.is_initialized()) {
                    g_eskf.get_output(g_output);
                    g_update_count++;

                    uint64_t now = eskf_time_us();

                    /* Publish attitude mỗi update */
                    publish_attitude(now);

                    /* Publish status ít hơn (~25 Hz) */
                    if (g_update_count % 40 == 0) {
                        publish_status(now);
                    }

                    /* Log lần đầu initialized */
                    if (g_update_count == 1) {
                        syslog(LOG_INFO,
                            "[eskf] Initialized! Roll=%+.1f Pitch=%+.1f deg\n",
                            (double)(g_output.roll * RAD_TO_DEG),
                            (double)(g_output.pitch * RAD_TO_DEG));
                    }
                }
            }
        } else {
            /* Không có data mới, sleep để không busy-loop */
            usleep(500);
        }
    }

    /*=====================================================================
     * PHASE 6: Cleanup
     *====================================================================*/
    g_is_running = false;

    uorb::orb_unsubscribe(g_imu_sub);
    if (g_att_pub)    uorb::orb_unadvertise(g_att_pub);
    if (g_status_pub) uorb::orb_unadvertise(g_status_pub);

    syslog(LOG_INFO, "[eskf] Stopped after %lu updates\n",
           (unsigned long)g_update_count);

    return 0;
}

/****************************************************************************
 * print_usage
 ****************************************************************************/

static void print_usage(void)
{
    printf("\n");
    printf("╔══════════════════════════════════════════════════╗\n");
    printf("║  ESKF - Error-State Kalman Filter Estimator     ║\n");
    printf("║  9-state attitude estimation (IMU-only)         ║\n");
    printf("╚══════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("Usage: eskf <command>\n");
    printf("\n");
    printf("Commands:\n");
    printf("  start    Khoi dong ESKF estimator task\n");
    printf("  stop     Dung estimator task\n");
    printf("  status   Xem trang thai va tu the hien tai\n");
    printf("\n");
    printf("Algorithm:\n");
    printf("  State:   [d_theta(3), d_bg(3), d_ba(3)] = 9-dim\n");
    printf("  Predict: Gyroscope propagation (quaternion)\n");
    printf("  Update:  Accelerometer gravity observation\n");
    printf("  Ref:     J. Sola, Quaternion kinematics for ESKF\n");
    printf("\n");
}

/****************************************************************************
 * print_status - In trạng thái chi tiết
 ****************************************************************************/

static void print_status(void)
{
    if (!g_is_running) {
        printf("[eskf] Not running\n");
        return;
    }

    printf("\n");
    printf("=== ESKF Status ===\n");
    printf("  State:       Running (%s)\n",
           g_eskf.is_initialized() ? "INITIALIZED" : "INITIALIZING");
    printf("  Updates:     %lu\n", (unsigned long)g_update_count);
    printf("\n");

    if (g_eskf.is_initialized()) {
        printf("  --- Attitude ---\n");
        printf("  Roll:  %+8.3f deg\n",
               (double)(g_output.roll * RAD_TO_DEG));
        printf("  Pitch: %+8.3f deg\n",
               (double)(g_output.pitch * RAD_TO_DEG));
        printf("  Yaw:   %+8.3f deg  (unobservable, drifts)\n",
               (double)(g_output.yaw * RAD_TO_DEG));
        printf("  Q:     [%+.4f, %+.4f, %+.4f, %+.4f]\n",
               (double)g_output.q[0], (double)g_output.q[1],
               (double)g_output.q[2], (double)g_output.q[3]);
        printf("\n");

        printf("  --- Gyro Bias [deg/s] ---\n");
        printf("  X: %+.4f  Y: %+.4f  Z: %+.4f\n",
               (double)(g_output.gyro_bias[0] * RAD_TO_DEG),
               (double)(g_output.gyro_bias[1] * RAD_TO_DEG),
               (double)(g_output.gyro_bias[2] * RAD_TO_DEG));
        printf("\n");

        printf("  --- Accel Bias [m/s2] ---\n");
        printf("  X: %+.4f  Y: %+.4f  Z: %+.4f\n",
               (double)g_output.accel_bias[0],
               (double)g_output.accel_bias[1],
               (double)g_output.accel_bias[2]);
        printf("\n");

        printf("  --- Covariance (std dev) ---\n");
        printf("  Att:   [%.4f, %.4f, %.4f] deg\n",
               (double)(sqrtf(g_output.att_cov[0]) * RAD_TO_DEG),
               (double)(sqrtf(g_output.att_cov[1]) * RAD_TO_DEG),
               (double)(sqrtf(g_output.att_cov[2]) * RAD_TO_DEG));
        printf("  Gbias: [%.6f, %.6f, %.6f] deg/s\n",
               (double)(sqrtf(g_output.gyro_bias_cov[0]) * RAD_TO_DEG),
               (double)(sqrtf(g_output.gyro_bias_cov[1]) * RAD_TO_DEG),
               (double)(sqrtf(g_output.gyro_bias_cov[2]) * RAD_TO_DEG));
        printf("  Abias: [%.6f, %.6f, %.6f] m/s2\n",
               (double)sqrtf(g_output.accel_bias_cov[0]),
               (double)sqrtf(g_output.accel_bias_cov[1]),
               (double)sqrtf(g_output.accel_bias_cov[2]));
    }

    printf("\n");
}

/****************************************************************************
 * eskf_main - Entry point (NuttX application)
 ****************************************************************************/

extern "C" int eskf_main(int argc, char *argv[])
{
    if (argc < 2) {
        print_usage();
        return 1;
    }

    const char *cmd = argv[1];

    /*---------------------------------------------------------------------
     * Command: start
     *--------------------------------------------------------------------*/
    if (strcmp(cmd, "start") == 0) {
        if (g_is_running) {
            printf("[eskf] Already running\n");
            return 0;
        }

        g_should_exit   = 0;
        g_update_count  = 0;
        g_predict_count = 0;
        g_update_skip   = 0;

        signal(SIGINT,  signal_handler);
        signal(SIGTERM, signal_handler);

        g_task_pid = task_create(
            "eskf",
            CONFIG_UAV_ESKF_PRIORITY,
            CONFIG_UAV_ESKF_STACKSIZE,
            eskf_thread_main,
            nullptr
        );

        if (g_task_pid < 0) {
            printf("[eskf] Failed to create task: %d\n", errno);
            return -errno;
        }

        printf("[eskf] Started ESKF 9-state estimator (pid=%d)\n", g_task_pid);
        return 0;
    }

    /*---------------------------------------------------------------------
     * Command: stop
     *--------------------------------------------------------------------*/
    if (strcmp(cmd, "stop") == 0) {
        if (!g_is_running) {
            printf("[eskf] Not running\n");
            return 0;
        }

        g_should_exit = 1;

        for (int i = 0; i < 20 && g_is_running; i++) {
            usleep(100000);
        }

        if (g_is_running) {
            printf("[eskf] Timeout waiting for task\n");
            return 1;
        }

        printf("[eskf] Stopped\n");
        return 0;
    }

    /*---------------------------------------------------------------------
     * Command: status
     *--------------------------------------------------------------------*/
    if (strcmp(cmd, "status") == 0) {
        print_status();
        return 0;
    }

    print_usage();
    return 1;
}
