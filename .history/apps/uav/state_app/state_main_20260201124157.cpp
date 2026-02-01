/****************************************************************************
 * apps/uav/state_app/state_main.cpp
 *
 * State Application - Quản lý trạng thái UAV
 *
 * MỤC ĐÍCH:
 * - Quản lý arming/disarming
 * - Preflight checks
 * - Failsafe detection
 * - State machine tổng thể
 *
 * THIẾT KẾ:
 * - Rate-based loop (50 Hz)
 * - Subscribe attitude/position từ estimator
 * - Publish vehicle_state cho controller
 *
 * TIMING:
 * - Loop rate: 50 Hz (20 ms period)
 * - Priority: 220 (thấp hơn estimator)
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sched.h>
#include <fcntl.h>
#include <errno.h>
#include <syslog.h>

#include <uav/uorb/uorb.hpp>
#include <uav/uorb/topics/vehicle_attitude.hpp>
#include <uav/uorb/topics/vehicle_local_position.hpp>
#include <uav/uorb/topics/sensor_imu.hpp>
#include <uav/uorb/topics/vehicle_state.hpp>

// GPIO character driver (/dev/gpio/led) để toggle PC0
#include <nuttx/ioexpander/gpio.h>

extern "C" {
#include <nuttx/timers/drv_hrt.h>
}

/****************************************************************************
 * Cấu hình
 ****************************************************************************/

#ifndef CONFIG_UAV_STATE_PRIORITY
#define CONFIG_UAV_STATE_PRIORITY   220
#endif

#ifndef CONFIG_UAV_STATE_STACKSIZE
#define CONFIG_UAV_STATE_STACKSIZE  4096
#endif

#define LOOP_RATE_HZ        50
#define LOOP_PERIOD_US      (1000000 / LOOP_RATE_HZ)

// Timeout thresholds
#define IMU_TIMEOUT_US      500000    // 500 ms
#define ATT_TIMEOUT_US      500000    // 500 ms

// Blink LED PC0 để kiểm tra RT sync
#define LED_BLINK_PERIOD_US 2000000   // 2 seconds

/****************************************************************************
 * State và Mode Definitions
 ****************************************************************************/

// UAV State Machine states
enum class UavState : uint8_t {
    UNINITIALIZED = 0,  // Chờ sensors
    STANDBY,            // Sẵn sàng arm
    ARMED,              // Motors enabled
    IN_FLIGHT,          // Đang bay
    EMERGENCY,          // Emergency landing/disarm
};

// Flight modes
enum class FlightMode : uint8_t {
    STABILIZE = 0,
    ALTITUDE,
    POSITION,
    AUTO,
    LAND,
    RTL,
};

// Chuyển state sang string để debug
static const char* state_to_string(UavState state)
{
    switch (state) {
        case UavState::UNINITIALIZED: return "UNINIT";
        case UavState::STANDBY:       return "STANDBY";
        case UavState::ARMED:         return "ARMED";
        case UavState::IN_FLIGHT:     return "IN_FLIGHT";
        case UavState::EMERGENCY:     return "EMERGENCY";
        default:                      return "UNKNOWN";
    }
}

static const char* mode_to_string(FlightMode mode)
{
    switch (mode) {
        case FlightMode::STABILIZE: return "STABILIZE";
        case FlightMode::ALTITUDE:  return "ALTITUDE";
        case FlightMode::POSITION:  return "POSITION";
        case FlightMode::AUTO:      return "AUTO";
        case FlightMode::LAND:      return "LAND";
        case FlightMode::RTL:       return "RTL";
        default:                    return "UNKNOWN";
    }
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile sig_atomic_t g_should_exit = 0;
static volatile bool g_is_running = false;
static pid_t g_task_pid = -1;

// Current state
static UavState g_uav_state = UavState::UNINITIALIZED;
static FlightMode g_flight_mode = FlightMode::STABILIZE;
static bool g_armed = false;

// Arm request (từ command hoặc RC)
static volatile bool g_arm_request = false;
static volatile bool g_disarm_request = false;

// Subscriptions
static int g_att_sub = -1;
static int g_pos_sub = -1;
static int g_imu_sub = -1;

// Publications
static uorb::orb_advert_t g_state_pub = nullptr;

// Pre-allocated messages
static vehicle_attitude_s g_att_data;
static vehicle_local_position_s g_pos_data;
static sensor_imu_s g_imu_data;
static vehicle_state_s g_state_msg;

// Timestamps của data cuối
static uint64_t g_last_imu_time = 0;
static uint64_t g_last_att_time = 0;

// Statistics
static uint32_t g_loop_count = 0;
static uint32_t g_state_changes = 0;

// LED PC0 (board registers /dev/gpio/led khi CONFIG_DEV_GPIO=y)
static int g_led_fd = -1;
static bool g_led_on = false;
static uint64_t g_led_next_toggle = 0;

/****************************************************************************
 * Signal Handler
 ****************************************************************************/

static void signal_handler(int signo)
{
    (void)signo;
    g_should_exit = 1;
}

/****************************************************************************
 * Preflight Checks - Kiểm tra trước khi arm
 ****************************************************************************/

struct PreflightResult {
    bool pass;
    const char* fail_reason;
};

static PreflightResult run_preflight_checks(void)
{
    PreflightResult result = {true, nullptr};
    uint64_t now = hrt_absolute_time();

    // Check 1: IMU data fresh?
    if (g_last_imu_time == 0) {
        result.pass = false;
        result.fail_reason = "No IMU data";
        return result;
    }
    if (now - g_last_imu_time > IMU_TIMEOUT_US) {
        result.pass = false;
        result.fail_reason = "IMU timeout";
        return result;
    }

    // Check 2: Attitude estimate valid?
    if (g_last_att_time == 0) {
        result.pass = false;
        result.fail_reason = "No attitude estimate";
        return result;
    }
    if (now - g_last_att_time > ATT_TIMEOUT_US) {
        result.pass = false;
        result.fail_reason = "Attitude timeout";
        return result;
    }

    // Check 3: Vehicle level? (tilt < 10 deg)
    float tilt_rad = sqrtf(g_att_data.roll * g_att_data.roll +
                           g_att_data.pitch * g_att_data.pitch);
    if (tilt_rad > 0.1745f) {  // 10 degrees
        result.pass = false;
        result.fail_reason = "Vehicle not level";
        return result;
    }

    // TODO: Thêm checks khác (battery, GPS, etc.)

    return result;
}

/****************************************************************************
 * Failsafe Check - Phát hiện lỗi runtime
 ****************************************************************************/

static bool check_failsafes(void)
{
    uint64_t now = hrt_absolute_time();
    bool failsafe = false;

    // Failsafe 1: IMU data loss
    if (g_last_imu_time > 0 && (now - g_last_imu_time > IMU_TIMEOUT_US)) {
        syslog(LOG_ERR, "[state] FAILSAFE: IMU data lost!\n");
        failsafe = true;
    }

    // Failsafe 2: Attitude estimate loss
    if (g_last_att_time > 0 && (now - g_last_att_time > ATT_TIMEOUT_US)) {
        syslog(LOG_ERR, "[state] FAILSAFE: Attitude lost!\n");
        failsafe = true;
    }

    return failsafe;
}

/****************************************************************************
 * State Machine Update
 ****************************************************************************/

static void update_state_machine(void)
{
    UavState new_state = g_uav_state;

    switch (g_uav_state) {
        case UavState::UNINITIALIZED:
            // Chờ sensors OK
            if (g_last_imu_time > 0 && g_last_att_time > 0) {
                new_state = UavState::STANDBY;
                syslog(LOG_INFO, "[state] Sensors OK, entering STANDBY\n");
            }
            break;

        case UavState::STANDBY:
            // Xử lý arm request
            if (g_arm_request) {
                g_arm_request = false;
                PreflightResult pf = run_preflight_checks();
                if (pf.pass) {
                    g_armed = true;
                    new_state = UavState::ARMED;
                    syslog(LOG_WARNING, "[state] ARMED!\n");
                } else {
                    syslog(LOG_WARNING, "[state] Arm rejected: %s\n", pf.fail_reason);
                }
            }
            break;

        case UavState::ARMED:
            // Check disarm request
            if (g_disarm_request) {
                g_disarm_request = false;
                g_armed = false;
                new_state = UavState::STANDBY;
                syslog(LOG_INFO, "[state] Disarmed\n");
            }
            // Check failsafes
            if (check_failsafes()) {
                g_armed = false;
                new_state = UavState::EMERGENCY;
                syslog(LOG_CRIT, "[state] EMERGENCY - failsafe triggered\n");
            }
            break;

        case UavState::IN_FLIGHT:
            // Check failsafes
            if (check_failsafes()) {
                g_armed = false;
                new_state = UavState::EMERGENCY;
            }
            // Check disarm (only if landed)
            if (g_disarm_request) {
                g_disarm_request = false;
                // TODO: Check if actually landed
                g_armed = false;
                new_state = UavState::STANDBY;
            }
            break;

        case UavState::EMERGENCY:
            // Reset về standby sau khi disarm
            if (!g_armed) {
                // Wait for operator to reset
                if (g_disarm_request) {
                    g_disarm_request = false;
                    new_state = UavState::STANDBY;
                    syslog(LOG_INFO, "[state] Emergency cleared\n");
                }
            }
            break;
    }

    // State transition
    if (new_state != g_uav_state) {
        syslog(LOG_INFO, "[state] %s -> %s\n",
               state_to_string(g_uav_state),
               state_to_string(new_state));
        g_uav_state = new_state;
        g_state_changes++;
    }
}

/****************************************************************************
 * Publish State
 ****************************************************************************/

static void publish_state(uint64_t timestamp)
{
    g_state_msg.timestamp_us = timestamp;
    g_state_msg.armed = g_armed;

    // Map internal state to message
    switch (g_uav_state) {
        case UavState::UNINITIALIZED:
            g_state_msg.nav_state = 0;
            g_state_msg.failsafe = false;
            break;
        case UavState::STANDBY:
            g_state_msg.nav_state = 1;
            g_state_msg.failsafe = false;
            break;
        case UavState::ARMED:
        case UavState::IN_FLIGHT:
            g_state_msg.nav_state = 2;
            g_state_msg.failsafe = false;
            break;
        case UavState::EMERGENCY:
            g_state_msg.nav_state = 255;
            g_state_msg.failsafe = true;
            break;
    }

    g_state_msg.flight_mode = static_cast<uint8_t>(g_flight_mode);

    if (g_state_pub) {
        uorb::orb_publish(ORB_ID(vehicle_state), g_state_pub, &g_state_msg);
    }
}

/****************************************************************************
 * Update Subscriptions
 ****************************************************************************/

static void update_subscriptions(void)
{
    bool updated;

    // IMU
    if (uorb::orb_check(g_imu_sub, &updated) == 0 && updated) {
        if (uorb::orb_copy(ORB_ID(sensor_imu), g_imu_sub, &g_imu_data) == 0) {
            g_last_imu_time = g_imu_data.timestamp_us;
        }
    }

    // Attitude
    if (uorb::orb_check(g_att_sub, &updated) == 0 && updated) {
        if (uorb::orb_copy(ORB_ID(vehicle_attitude), g_att_sub, &g_att_data) == 0) {
            g_last_att_time = g_att_data.timestamp_us;
        }
    }

    // Position (optional)
    if (uorb::orb_check(g_pos_sub, &updated) == 0 && updated) {
        uorb::orb_copy(ORB_ID(vehicle_local_position), g_pos_sub, &g_pos_data);
    }
}

/****************************************************************************
 * state_thread_main - Main loop
 ****************************************************************************/

static int state_thread_main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    //=========================================================================
    // PHASE 1: Set priority
    //=========================================================================

    struct sched_param param;
    param.sched_priority = CONFIG_UAV_STATE_PRIORITY;
    sched_setscheduler(0, SCHED_FIFO, &param);

    //=========================================================================
    // PHASE 2: Subscribe
    //=========================================================================

    syslog(LOG_INFO, "[state] Subscribing...\n");

    g_imu_sub = uorb::orb_subscribe(ORB_ID(sensor_imu));
    g_att_sub = uorb::orb_subscribe(ORB_ID(vehicle_attitude));
    g_pos_sub = uorb::orb_subscribe(ORB_ID(vehicle_local_position));

    //=========================================================================
    // PHASE 3: Advertise
    //=========================================================================

    memset(&g_state_msg, 0, sizeof(g_state_msg));
    g_state_pub = uorb::orb_advertise(ORB_ID(vehicle_state), &g_state_msg);

    //=========================================================================
    // PHASE 3.5: Open GPIO LED device (PC0)
    //=========================================================================

    g_led_fd = open("/dev/gpio/led", O_RDWR);
    if (g_led_fd < 0) {
        syslog(LOG_WARNING, "[state] LED device not available (/dev/gpio/led), enable CONFIG_DEV_GPIO\n");
    } else {
        (void)ioctl(g_led_fd, GPIOC_WRITE, 0);
        g_led_on = false;
        g_led_next_toggle = hrt_absolute_time() + LED_BLINK_PERIOD_US;
    }

    //=========================================================================
    // PHASE 4: Init state
    //=========================================================================

    g_uav_state = UavState::UNINITIALIZED;
    g_flight_mode = FlightMode::STABILIZE;
    g_armed = false;
    g_is_running = true;

    syslog(LOG_INFO, "[state] Running at %d Hz\n", LOOP_RATE_HZ);

    //=========================================================================
    // PHASE 5: Main loop
    //=========================================================================

    while (!g_should_exit) {
        uint64_t loop_start = hrt_absolute_time();

        // Read latest data
        update_subscriptions();

        // Run state machine
        update_state_machine();

        // Publish state
        publish_state(loop_start);

        // Blink LED mỗi 2 giây để check timing đồng bộ
        if (g_led_fd >= 0) {
            if (loop_start >= g_led_next_toggle) {
                // Catch-up để tránh drift khi bị trễ
                do {
                    g_led_next_toggle += LED_BLINK_PERIOD_US;
                } while (loop_start >= g_led_next_toggle);

                g_led_on = !g_led_on;
                (void)ioctl(g_led_fd, GPIOC_WRITE, g_led_on ? 1 : 0);
            }
        }

        g_loop_count++;

        // Sleep to maintain rate
        uint64_t elapsed = hrt_absolute_time() - loop_start;
        if (elapsed < LOOP_PERIOD_US) {
            usleep(LOOP_PERIOD_US - elapsed);
        }
    }

    //=========================================================================
    // PHASE 6: Cleanup
    //=========================================================================

    g_is_running = false;
    g_armed = false;

    uorb::orb_unsubscribe(g_imu_sub);
    uorb::orb_unsubscribe(g_att_sub);
    uorb::orb_unsubscribe(g_pos_sub);

    if (g_state_pub) uorb::orb_unadvertise(g_state_pub);

    if (g_led_fd >= 0) {
        (void)ioctl(g_led_fd, GPIOC_WRITE, 0);
        close(g_led_fd);
        g_led_fd = -1;
    }

    syslog(LOG_INFO, "[state] Stopped after %lu loops, %lu state changes\n",
           (unsigned long)g_loop_count, (unsigned long)g_state_changes);

    return 0;
}

/****************************************************************************
 * print_usage
 ****************************************************************************/

static void print_usage(void)
{
    printf("Usage: state <command>\n");
    printf("\nCommands:\n");
    printf("  start     Khởi động state manager\n");
    printf("  stop      Dừng state manager\n");
    printf("  status    Xem trạng thái hiện tại\n");
    printf("  arm       Arm UAV (khi đủ điều kiện)\n");
    printf("  disarm    Disarm UAV\n");
}

/****************************************************************************
 * state_main - Entry point
 ****************************************************************************/

extern "C" int state_main(int argc, char *argv[])
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
            printf("[state] Already running\n");
            return 0;
        }

        g_should_exit = 0;
        g_loop_count = 0;
        g_state_changes = 0;
        g_arm_request = false;
        g_disarm_request = false;

        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);

        g_task_pid = task_create(
            "state",
            CONFIG_UAV_STATE_PRIORITY,
            CONFIG_UAV_STATE_STACKSIZE,
            state_thread_main,
            nullptr
        );

        if (g_task_pid < 0) {
            printf("[state] Failed to create task: %d\n", errno);
            return -errno;
        }

        printf("[state] Started (pid=%d)\n", g_task_pid);
        return 0;
    }

    //-------------------------------------------------------------------------
    // Command: stop
    //-------------------------------------------------------------------------

    if (strcmp(cmd, "stop") == 0) {
        if (!g_is_running) {
            printf("[state] Not running\n");
            return 0;
        }

        // Force disarm trước khi stop
        if (g_armed) {
            printf("[state] WARNING: Forcing disarm!\n");
            g_armed = false;
        }

        g_should_exit = 1;

        for (int i = 0; i < 20 && g_is_running; i++) {
            usleep(100000);
        }

        if (g_is_running) {
            printf("[state] Timeout waiting for task\n");
            return 1;
        }

        printf("[state] Stopped\n");
        return 0;
    }

    //-------------------------------------------------------------------------
    // Command: status
    //-------------------------------------------------------------------------

    if (strcmp(cmd, "status") == 0) {
        if (!g_is_running) {
            printf("[state] Not running\n");
            return 0;
        }

        printf("[state] Status:\n");
        printf("  State:       %s\n", state_to_string(g_uav_state));
        printf("  Mode:        %s\n", mode_to_string(g_flight_mode));
        printf("  Armed:       %s\n", g_armed ? "YES" : "NO");
        printf("  Loops:       %lu\n", (unsigned long)g_loop_count);
        printf("  Transitions: %lu\n", (unsigned long)g_state_changes);

        if (g_last_att_time > 0) {
            printf("  Roll:  %+7.2f deg\n", (double)(g_att_data.roll * 57.2957795f));
            printf("  Pitch: %+7.2f deg\n", (double)(g_att_data.pitch * 57.2957795f));
            printf("  Yaw:   %+7.2f deg\n", (double)(g_att_data.yaw * 57.2957795f));
        }

        return 0;
    }

    //-------------------------------------------------------------------------
    // Command: arm
    //-------------------------------------------------------------------------

    if (strcmp(cmd, "arm") == 0) {
        if (!g_is_running) {
            printf("[state] Not running\n");
            return 1;
        }

        if (g_armed) {
            printf("[state] Already armed\n");
            return 0;
        }

        printf("[state] Arm request sent\n");
        g_arm_request = true;
        return 0;
    }

    //-------------------------------------------------------------------------
    // Command: disarm
    //-------------------------------------------------------------------------

    if (strcmp(cmd, "disarm") == 0) {
        if (!g_is_running) {
            printf("[state] Not running\n");
            return 1;
        }

        if (!g_armed) {
            printf("[state] Not armed\n");
            return 0;
        }

        printf("[state] Disarm request sent\n");
        g_disarm_request = true;
        return 0;
    }

    print_usage();
    return 1;
}
