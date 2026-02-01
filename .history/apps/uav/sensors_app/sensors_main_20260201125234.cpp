/****************************************************************************
 * apps/uav/sensors_app/sensors_main.cpp
 *
 * Sensors Application - Entry Point
 *
 * MỤC ĐÍCH:
 * - Thu thập dữ liệu từ tất cả cảm biến
 * - Publish lên uORB topics
 * - Chạy ở priority cao nhất để đảm bảo timing
 *
 * THIẾT KẾ:
 * - Main loop chạy @ 1kHz (IMU rate)
 * - Các cảm biến khác được poll ở rate thấp hơn (divider)
 * - Tất cả hardware I/O tập trung trong 1 task
 * - Không dynamic allocation trong loop
 *
 * FLOW:
 *   1. Parse arguments (start/stop/status)
 *   2. Init hardware drivers
 *   3. Advertise uORB topics
 *   4. Enter main loop
 *   5. Cleanup on exit
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

#include <nuttx/clock.h>
#include <time.h>

#include <uav/uorb/uorb.hpp>
#include <uav/uorb/topics/sensor_imu.hpp>

static inline uint64_t uav_time_us(void)
{
    struct timespec ts;
    clock_systime_timespec(&ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/****************************************************************************
 * Cấu hình - Có thể override qua Kconfig
 ****************************************************************************/

#ifndef CONFIG_UAV_SENSORS_PRIORITY
#define CONFIG_UAV_SENSORS_PRIORITY     250
#endif

#ifndef CONFIG_UAV_SENSORS_STACKSIZE
#define CONFIG_UAV_SENSORS_STACKSIZE    4096
#endif

#ifndef CONFIG_UAV_IMU_RATE_HZ
#define CONFIG_UAV_IMU_RATE_HZ          1000
#endif

#ifndef CONFIG_UAV_NUM_IMUS
#define CONFIG_UAV_NUM_IMUS             4
#endif

/****************************************************************************
 * Private Data - Static allocation
 *
 * Tất cả data structures phải static để:
 * 1. Không malloc trong runtime
 * 2. Tồn tại xuyên suốt lifetime của app
 * 3. Predictable memory layout
 ****************************************************************************/

// Task control
static volatile sig_atomic_t g_should_exit = 0;
static volatile bool g_is_running = false;
static pid_t g_task_pid = -1;

// uORB publishers
static uorb::orb_advert_t g_imu_pub[CONFIG_UAV_NUM_IMUS] = {nullptr};

// Pre-allocated message buffers (reused mỗi cycle)
static sensor_imu_s g_imu_msg[CONFIG_UAV_NUM_IMUS];

// Statistics
static uint32_t g_loop_count = 0;
static uint32_t g_deadline_misses = 0;

/****************************************************************************
 * Signal Handler
 *
 * Được gọi khi nhận SIGINT/SIGTERM.
 * Chỉ set flag, không làm gì phức tạp trong signal context.
 ****************************************************************************/

static void signal_handler(int signo)
{
    (void)signo;
    g_should_exit = 1;
}

/****************************************************************************
 * poll_imus - Đọc tất cả IMU và publish
 *
 * @param now_us Timestamp hiện tại
 *
 * THIẾT KẾ:
 * - Đọc từng IMU theo thứ tự
 * - Skip IMU bị lỗi
 * - Publish ngay sau khi đọc (minimize latency)
 ****************************************************************************/

static void poll_imus(uint64_t now_us)
{
    // TODO: Thay bằng real driver khi integrate
    // Hiện tại dùng dummy data để test framework

    for (int i = 0; i < CONFIG_UAV_NUM_IMUS; i++) {
        // Simulate IMU read
        g_imu_msg[i].timestamp_us = now_us;
        g_imu_msg[i].instance = i;

        // Dummy data (sẽ thay bằng real driver)
        g_imu_msg[i].accel[0] = 0.0f;
        g_imu_msg[i].accel[1] = 0.0f;
        g_imu_msg[i].accel[2] = -9.81f;  // Gravity
        g_imu_msg[i].gyro[0] = 0.0f;
        g_imu_msg[i].gyro[1] = 0.0f;
        g_imu_msg[i].gyro[2] = 0.0f;
        g_imu_msg[i].temperature = 25.0f;

        // Publish
        if (g_imu_pub[i]) {
            uorb::orb_publish(ORB_ID(sensor_imu), g_imu_pub[i], &g_imu_msg[i]);
        }
    }
}

/****************************************************************************
 * sensors_thread_main - Main loop của sensors task
 *
 * Được spawn bởi task_create().
 * Chạy cho đến khi g_should_exit = true.
 ****************************************************************************/

static int sensors_thread_main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    //=========================================================================
    // PHASE 1: Set realtime priority
    //
    // Dùng SCHED_FIFO để có deterministic scheduling.
    // Priority cao nhất trong hệ thống.
    //=========================================================================

    struct sched_param param;
    param.sched_priority = CONFIG_UAV_SENSORS_PRIORITY;
    int ret = sched_setscheduler(0, SCHED_FIFO, &param);
    if (ret < 0) {
        syslog(LOG_WARNING, "[sensors] Failed to set FIFO scheduler: %d\n", errno);
    }

    //=========================================================================
    // PHASE 2: Init hardware drivers
    //
    // TODO: Integrate real ICM42688P driver ở đây
    // Hiện tại skip để test uORB framework
    //=========================================================================

    syslog(LOG_INFO, "[sensors] Initializing %d IMUs...\n", CONFIG_UAV_NUM_IMUS);

    // Placeholder: Trong real implementation, init SPI và sensors ở đây
    int num_imus = CONFIG_UAV_NUM_IMUS;

    syslog(LOG_INFO, "[sensors] Found %d IMUs\n", num_imus);

    //=========================================================================
    // PHASE 3: Advertise uORB topics
    //
    // Mỗi IMU có một topic instance riêng.
    //=========================================================================

    for (int i = 0; i < num_imus; i++) {
        memset(&g_imu_msg[i], 0, sizeof(sensor_imu_s));
        g_imu_msg[i].instance = i;

        g_imu_pub[i] = uorb::orb_advertise_multi(
            ORB_ID(sensor_imu),
            &g_imu_msg[i],
            i
        );

        if (!g_imu_pub[i]) {
            syslog(LOG_ERR, "[sensors] Failed to advertise IMU[%d]\n", i);
        }
    }

    //=========================================================================
    // PHASE 4: Main loop - Rate-controlled
    //
    // TIMING STRATEGY:
    // - Tính next_time = now + period
    // - Sleep đến gần next_time (trừ margin 50µs)
    // - Spin wait phần còn lại cho precision
    // - Detect deadline miss
    //=========================================================================

    g_is_running = true;
    syslog(LOG_INFO, "[sensors] Running @ %d Hz\n", CONFIG_UAV_IMU_RATE_HZ);

    const uint32_t loop_period_us = 1000000 / CONFIG_UAV_IMU_RATE_HZ;
    uint64_t next_time = uav_time_us();

    while (!g_should_exit) {
        uint64_t now = uav_time_us();

        //---------------------------------------------------------------------
        // Poll sensors
        //---------------------------------------------------------------------

        poll_imus(now);

        //---------------------------------------------------------------------
        // Statistics
        //---------------------------------------------------------------------

        g_loop_count++;

        //---------------------------------------------------------------------
        // Sleep until next cycle
        //
        // Strategy:
        // 1. usleep() cho phần lớn thời gian
        // 2. Spin wait 50µs cuối cho precision
        //---------------------------------------------------------------------

        next_time += loop_period_us;
        int64_t sleep_us = (int64_t)next_time - (int64_t)uav_time_us();

        if (sleep_us > 100) {
            // Sleep bớt 50µs để wake sớm
            usleep(sleep_us - 50);
        } else if (sleep_us < -(int64_t)loop_period_us) {
            // Deadline miss > 1 full period - reset timing
            g_deadline_misses++;
            next_time = uav_time_us() + loop_period_us;

            if (g_deadline_misses % 100 == 1) {
                syslog(LOG_WARNING, "[sensors] Deadline miss #%lu\n",
                       (unsigned long)g_deadline_misses);
            }
        }

        // Spin wait for precision (optional, có thể bỏ nếu không cần)
        while (uav_time_us() < next_time) {
            // Busy wait
        }
    }

    //=========================================================================
    // PHASE 5: Cleanup
    //=========================================================================

    g_is_running = false;

    for (int i = 0; i < CONFIG_UAV_NUM_IMUS; i++) {
        if (g_imu_pub[i]) {
            uorb::orb_unadvertise(g_imu_pub[i]);
            g_imu_pub[i] = nullptr;
        }
    }

    syslog(LOG_INFO, "[sensors] Stopped after %lu loops, %lu deadline misses\n",
           (unsigned long)g_loop_count, (unsigned long)g_deadline_misses);

    return 0;
}

/****************************************************************************
 * print_usage - In hướng dẫn sử dụng
 ****************************************************************************/

static void print_usage(void)
{
    printf("Usage: sensors <command>\n");
    printf("\nCommands:\n");
    printf("  start     Khởi động sensors task\n");
    printf("  stop      Dừng sensors task\n");
    printf("  status    Xem trạng thái\n");
}

/****************************************************************************
 * sensors_main - Entry point (được gọi từ NSH)
 *
 * Xử lý các commands: start, stop, status
 ****************************************************************************/

extern "C" int sensors_main(int argc, char *argv[])
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
            printf("[sensors] Already running\n");
            return 0;
        }

        // Reset state
        g_should_exit = 0;
        g_loop_count = 0;
        g_deadline_misses = 0;

        // Install signal handlers
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);

        // Create task
        g_task_pid = task_create(
            "sensors",
            CONFIG_UAV_SENSORS_PRIORITY,
            CONFIG_UAV_SENSORS_STACKSIZE,
            sensors_thread_main,
            nullptr
        );

        if (g_task_pid < 0) {
            printf("[sensors] Failed to create task: %d\n", errno);
            return -errno;
        }

        printf("[sensors] Started (pid=%d)\n", g_task_pid);
        return 0;
    }

    //-------------------------------------------------------------------------
    // Command: stop
    //-------------------------------------------------------------------------

    if (strcmp(cmd, "stop") == 0) {
        if (!g_is_running) {
            printf("[sensors] Not running\n");
            return 0;
        }

        g_should_exit = 1;

        // Wait for task to exit (timeout 2s)
        for (int i = 0; i < 20 && g_is_running; i++) {
            usleep(100000);  // 100ms
        }

        if (g_is_running) {
            printf("[sensors] Timeout waiting for task to stop\n");
            return 1;
        }

        printf("[sensors] Stopped\n");
        return 0;
    }

    //-------------------------------------------------------------------------
    // Command: status
    //-------------------------------------------------------------------------

    if (strcmp(cmd, "status") == 0) {
        if (!g_is_running) {
            printf("[sensors] Not running\n");
            return 0;
        }

        printf("[sensors] Running\n");
        printf("  Loop count:      %lu\n", (unsigned long)g_loop_count);
        printf("  Deadline misses: %lu\n", (unsigned long)g_deadline_misses);
        printf("  Rate:            %d Hz\n", CONFIG_UAV_IMU_RATE_HZ);
        printf("  IMUs:            %d\n", CONFIG_UAV_NUM_IMUS);

        return 0;
    }

    //-------------------------------------------------------------------------
    // Unknown command
    //-------------------------------------------------------------------------

    print_usage();
    return 1;
}
