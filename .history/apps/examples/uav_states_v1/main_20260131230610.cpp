/****************************************************************************
 * apps/examples/uav_states_v1/main.cpp
 *
 * Ứng dụng ước lượng tư thế UAV - Chương trình chính (Điều phối trung tâm)
 *
 * TỔNG QUAN KIẾN TRÚC:
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
 * LUỒNG DỮ LIỆU (Pub/Sub):
 * 1. ImuModule chạy thread riêng @ 100Hz, đọc cảm biến.
 * 2. ImuModule publish sensor_imu_s lên topics.
 * 3. Main loop gọi AttitudeModule.update() định kỳ.
 * 4. AttitudeModule subscribe sensor_imu, tính tư thế.
 * 5. AttitudeModule publish vehicle_attitude_s.
 * 6. Main loop đọc attitude và hiển thị bằng syslog.
 *
 * LÝ DO CHỌN THIẾT KẾ:
 * - I/O cảm biến tách riêng một thread -> tránh xung đột SPI.
 * - Estimator chạy trong main thread -> dễ debug.
 * - Hiển thị qua syslog -> không blocking, tránh lag console.
 * - Mỗi module độc lập -> dễ mở rộng.
 *
 * MỞ RỘNG TƯƠNG LAI:
 * - Thêm MagModule cho BMM150.
 * - Thêm BaroModule cho MS5611.
 * - Thêm LoggerModule ghi SD card.
 * - Thêm EKF2 cho ước lượng trạng thái đầy đủ.
 * - Thêm ControllerModule cho điều khiển tư thế.
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
 * Cấu hình
 ****************************************************************************/

#define MAIN_LOOP_RATE_HZ   100   // Main loop frequency
#define DISPLAY_RATE_HZ     10    // Attitude display frequency

static constexpr float RAD2DEG = 57.2957795f;

/****************************************************************************
 * Trạng thái toàn cục
 ****************************************************************************/

static volatile bool g_running = true;
static modules::sensors::ImuModule g_imu_module;
static modules::estimator::AttitudeModule g_attitude_module;

/****************************************************************************
 * Signal Handler - Dừng sạch khi Ctrl+C
 ****************************************************************************/

static void signal_handler(int signo)
{
    (void)signo;
    g_running = false;
    g_imu_module.request_stop();
}

/****************************************************************************
 * Hàm hiển thị - Dùng syslog để output không bị blocking
 *
 * syslog bất đồng bộ và không chặn thread gọi,
 * phù hợp cho hiển thị realtime mà không ảnh hưởng timing.
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
 * @brief Hiển thị attitude bằng syslog (không blocking)
 *
 * Dùng syslog thay printf vì:
 * 1. syslog có buffer và ghi bất đồng bộ.
 * 2. printf flush ra serial có thể chặn ~10ms.
 * 3. Blocking làm lệch timing và ảnh hưởng đọc cảm biến.
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
 * Điểm vào chương trình chính
 *
 * Nhiệm vụ:
 * 1. Khởi tạo các module.
 * 2. Khởi chạy thread cảm biến.
 * 3. Chạy main loop (gọi estimator, hiển thị).
 * 4. Xử lý dừng chương trình.
 ****************************************************************************/

extern "C" int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    // Cài đặt signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    //=========================================================================
    // GIAI ĐOẠN 1: Khởi tạo module cảm biến
    //=========================================================================

    int num_imus = g_imu_module.init();
    if (num_imus == 0) {
        syslog(LOG_ERR, "[main] ERROR: No IMU sensors available!\n");
        printf("ERROR: No IMU sensors available!\n");
        return 1;
    }

    //=========================================================================
    // GIAI ĐOẠN 2: Hiệu chuẩn cảm biến
    //=========================================================================

    if (!g_imu_module.calibrate()) {
        syslog(LOG_ERR, "[main] ERROR: Calibration failed!\n");
        printf("ERROR: Calibration failed!\n");
        return 1;
    }

    //=========================================================================
    // GIAI ĐOẠN 3: Khởi tạo module estimator
    //
    // Truyền IMU topics từ sensor module sang estimator module.
    // Đây là điểm kết nối pub/sub.
    //=========================================================================

    // Lấy con trỏ tới IMU topics (mỗi sensor một topic)
    uorb::Topic<uorb::sensor_imu_s, 8>* imu_topics[modules::sensors::MAX_IMUS];
    for (int i = 0; i < modules::sensors::MAX_IMUS; i++) {
        if (g_imu_module.is_active(i)) {
            imu_topics[i] = &g_imu_module.get_topic(i);
        } else {
            imu_topics[i] = nullptr;
        }
    }

    // Khởi tạo estimator với IMU topics
    g_attitude_module.init(imu_topics[0], num_imus);

    //=========================================================================
    // GIAI ĐOẠN 4: Khởi chạy thread cảm biến
    //
    // Thread cảm biến chạy độc lập, publish lên topics.
    // Main thread đọc topics thông qua estimator.
    //=========================================================================

    if (g_imu_module.start() != 0) {
        syslog(LOG_ERR, "[main] ERROR: Failed to start sensor thread!\n");
        printf("ERROR: Failed to start sensor thread!\n");
        return 1;
    }

    //=========================================================================
    // GIAI ĐOẠN 5: Main Loop
    //
    // - Chạy ở MAIN_LOOP_RATE_HZ.
    // - Gọi estimator.update() mỗi chu kỳ.
    // - Hiển thị attitude ở DISPLAY_RATE_HZ.
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
        // Cập nhật estimator (nhận IMU topics, xuất attitude)
        //---------------------------------------------------------------------
        g_attitude_module.update();

        //---------------------------------------------------------------------
        // Hiển thị ở tần số thấp hơn
        //---------------------------------------------------------------------
        loop_count++;
        if (loop_count % display_divider == 0) {
            uint32_t elapsed = (hrt_absolute_time() - start_time) / 1000000;
            display_attitude(elapsed, loop_count / display_divider);
        }

        //---------------------------------------------------------------------
        // Ngủ tới chu kỳ tiếp theo
        //---------------------------------------------------------------------
        next_time += loop_period_us;
        int64_t sleep_us = (int64_t)next_time - (int64_t)hrt_absolute_time();
        if (sleep_us > 0) {
            usleep(sleep_us);
        } else {
            // Quá thời gian - reset timing
            next_time = hrt_absolute_time();
        }
    }

    //=========================================================================
    // GIAI ĐOẠN 6: Dừng chương trình
    //=========================================================================

    g_imu_module.stop();
    syslog(LOG_INFO, "[main] Stopped.\n");
    printf("\nStopped.\n");

    return 0;
}
