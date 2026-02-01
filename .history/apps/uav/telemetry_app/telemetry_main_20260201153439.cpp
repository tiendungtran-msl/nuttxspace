/****************************************************************************
 * apps/uav/telemetry_app/telemetry_main.cpp
 *
 * UAV TELEMETRY APPLICATION
 *
 * MỤC ĐÍCH:
 * - Subscribe các uORB topic cần thiết
 * - Đóng gói dữ liệu vào binary packet 128 bytes
 * - Gửi qua UART với DMA (non-blocking)
 * - Chạy ở priority thấp, không ảnh hưởng realtime tasks
 *
 * TIMING:
 * - Rate: 100 Hz (10ms period)
 * - UART: 921600 baud
 * - Packet: 128 bytes → ~1.4ms transfer time
 *
 * SỬ DỤNG:
 *   telemetry start    - Khởi động telemetry
 *   telemetry stop     - Dừng telemetry
 *   telemetry status   - Xem trạng thái
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sched.h>
#include <syslog.h>
#include <termios.h>

#include <nuttx/clock.h>

/* Telemetry packet */
extern "C" {
#include "telemetry_packet.h"
}

/* UAV Platform */
#include <uav/lib/platform/hrt.h>

/* uORB */
#include <uav/uorb/uorb.hpp>
#include <uav/uorb/topics/sensor_imu.hpp>
#include <uav/uorb/topics/sensor_combined.hpp>
#include <uav/uorb/topics/vehicle_attitude.hpp>
#include <uav/uorb/topics/system_status.hpp>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_UAV_TELEMETRY_PRIORITY
#define CONFIG_UAV_TELEMETRY_PRIORITY   100     /* Low priority */
#endif

#ifndef CONFIG_UAV_TELEMETRY_STACKSIZE
#define CONFIG_UAV_TELEMETRY_STACKSIZE  4096
#endif

#ifndef CONFIG_UAV_TELEMETRY_RATE_HZ
#define CONFIG_UAV_TELEMETRY_RATE_HZ    100     /* 100 Hz = 10ms */
#endif

#ifndef CONFIG_UAV_TELEMETRY_UART_DEV
#define CONFIG_UAV_TELEMETRY_UART_DEV   "/dev/ttyS1"
#endif

#ifndef CONFIG_UAV_TELEMETRY_BAUDRATE
#define CONFIG_UAV_TELEMETRY_BAUDRATE   921600
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct TelemetryContext
{
    /* UART file descriptor */
    int uart_fd;

    /* uORB subscriptions */
    int combined_sub;
    int status_sub;
    int attitude_sub;
    int imu_sub;

    /* Pre-allocated packet */
    struct telemetry_packet_s packet;

    /* Sequence number */
    uint16_t sequence;

    /* Task state */
    volatile bool should_exit;
    volatile bool is_running;
    pid_t task_pid;

    /* Statistics */
    uint32_t packets_sent;
    uint32_t send_errors;
    uint32_t loop_count;
    uint64_t last_send_time;

    /* Constructor */
    TelemetryContext() :
        uart_fd(-1),
        combined_sub(-1),
        status_sub(-1),
        attitude_sub(-1),
        imu_sub(-1),
        sequence(0),
        should_exit(false),
        is_running(false),
        task_pid(-1),
        packets_sent(0),
        send_errors(0),
        loop_count(0),
        last_send_time(0)
    {
        memset(&packet, 0, sizeof(packet));
    }

    void reset()
    {
        uart_fd = -1;
        combined_sub = -1;
        status_sub = -1;
        attitude_sub = -1;
        imu_sub = -1;
        sequence = 0;
        should_exit = false;
        is_running = false;
        task_pid = -1;
        packets_sent = 0;
        send_errors = 0;
        loop_count = 0;
        last_send_time = 0;
        memset(&packet, 0, sizeof(packet));
    }
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static TelemetryContext g_telem_ctx;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief Configure UART for telemetry
 */
static int configure_uart(int fd, int baudrate)
{
    struct termios tio;

    if (tcgetattr(fd, &tio) < 0)
    {
        syslog(LOG_ERR, "[telemetry] tcgetattr failed: %d\n", errno);
        return -1;
    }

    /* Raw mode - no processing */
    cfmakeraw(&tio);

    /* Set baudrate */
    speed_t speed;
    switch (baudrate)
    {
        case 115200:  speed = B115200;  break;
        case 230400:  speed = B230400;  break;
        case 460800:  speed = B460800;  break;
        case 921600:  speed = B921600;  break;
        default:      speed = B115200;  break;
    }
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    /* 8N1 */
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;

    /* No flow control */
    tio.c_cflag &= ~CRTSCTS;

    /* Enable receiver, local mode */
    tio.c_cflag |= (CLOCAL | CREAD);

    /* Non-blocking */
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) < 0)
    {
        syslog(LOG_ERR, "[telemetry] tcsetattr failed: %d\n", errno);
        return -1;
    }

    return 0;
}

/**
 * @brief Fill packet with sensor data from uORB
 */
static void fill_packet_data(TelemetryContext *ctx)
{
    struct telemetry_packet_s *pkt = &ctx->packet;

    /* Initialize packet structure */
    telem_packet_init(pkt);

    /*=========================================================================
     * IMU DATA - từ sensor_combined topic
     *=========================================================================*/

    sensor_combined_s combined;
    if (ctx->combined_sub >= 0)
    {
        bool updated = false;
        uorb::orb_check(ctx->combined_sub, &updated);

        if (updated && uorb::orb_copy(ORB_ID(sensor_combined), ctx->combined_sub, &combined) == 0)
        {
            pkt->imu.gyro_x = combined.gyro[0];
            pkt->imu.gyro_y = combined.gyro[1];
            pkt->imu.gyro_z = combined.gyro[2];

            pkt->imu.accel_x = combined.accel[0];
            pkt->imu.accel_y = combined.accel[1];
            pkt->imu.accel_z = combined.accel[2];

            pkt->imu.temperature = combined.temperature;

            /* Attitude từ combined (nếu có) */
            pkt->attitude.qw = combined.q[0];
            pkt->attitude.qx = combined.q[1];
            pkt->attitude.qy = combined.q[2];
            pkt->attitude.qz = combined.q[3];

            pkt->attitude.roll = combined.roll;
            pkt->attitude.pitch = combined.pitch;
            pkt->attitude.yaw = combined.yaw;
        }
    }

    /*=========================================================================
     * SYSTEM STATUS
     *=========================================================================*/

    system_status_s status;
    if (ctx->status_sub >= 0)
    {
        bool updated = false;
        uorb::orb_check(ctx->status_sub, &updated);

        if (updated && uorb::orb_copy(ORB_ID(system_status), ctx->status_sub, &status) == 0)
        {
            pkt->status.health_level = status.health_level;
            pkt->status.healthy_imus = status.healthy_imus;

            /* Build sensor flags */
            uint8_t flags = 0;
            if (status.healthy_imus > 0) flags |= SENSOR_IMU_OK;
            if (status.baro_ok)          flags |= SENSOR_BARO_OK;
            if (status.mag_ok)           flags |= SENSOR_MAG_OK;
            if (status.gps_ok)           flags |= SENSOR_GPS_OK;
            if (status.ekf_ok)           flags |= SENSOR_EKF_OK;
            if (status.timebase_ok)      flags |= SENSOR_TIMEBASE_OK;
            pkt->status.sensor_flags = flags;

            /* CPU load: deadline_misses as proxy for now */
            pkt->status.cpu_load = (uint16_t)(status.deadline_misses & 0xFFFF);
        }
    }

    /*=========================================================================
     * MAG / BARO / GPS - TODO: Add subscriptions khi có topics
     *=========================================================================*/

    /* Placeholder - sẽ fill khi có mag/baro/gps topics */
    pkt->mag.mag_x = 0.0f;
    pkt->mag.mag_y = 0.0f;
    pkt->mag.mag_z = 0.0f;

    pkt->baro.pressure = 101325.0f;  /* Standard pressure */
    pkt->baro.altitude = 0.0f;

    pkt->gps.latitude = 0;
    pkt->gps.longitude = 0;
    pkt->gps.altitude_msl = 0;
    pkt->gps.fix_type = GPS_FIX_NONE;
    pkt->gps.satellites = 0;

    /*=========================================================================
     * LOOP COUNTER
     *=========================================================================*/

    pkt->status.loop_count = ctx->loop_count;
}

/**
 * @brief Send packet over UART
 */
static int send_packet(TelemetryContext *ctx)
{
    uint64_t now_us = hrt_absolute_time();

    /* Fill packet with current data */
    fill_packet_data(ctx);

    /* Finalize packet with CRC */
    telem_packet_finalize(&ctx->packet, ctx->sequence, (uint32_t)now_us);

    /* Send over UART */
    ssize_t written = write(ctx->uart_fd,
                            &ctx->packet,
                            TELEM_PACKET_SIZE);

    if (written != TELEM_PACKET_SIZE)
    {
        ctx->send_errors++;
        return -1;
    }

    /* Update stats */
    ctx->packets_sent++;
    ctx->sequence++;
    ctx->last_send_time = now_us;

    return 0;
}

/**
 * @brief Telemetry thread main
 */
static int telemetry_thread_main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    TelemetryContext *ctx = &g_telem_ctx;
    int ret;

    syslog(LOG_INFO, "[telemetry] Thread starting...\n");

    /*=========================================================================
     * PHASE 1: Set low priority
     *=========================================================================*/

    struct sched_param param;
    param.sched_priority = CONFIG_UAV_TELEMETRY_PRIORITY;
    ret = sched_setscheduler(0, SCHED_RR, &param);
    if (ret < 0)
    {
        syslog(LOG_WARNING, "[telemetry] Failed to set priority: %d\n", errno);
    }

    /*=========================================================================
     * PHASE 2: Open UART
     *=========================================================================*/

    ctx->uart_fd = open(CONFIG_UAV_TELEMETRY_UART_DEV, O_RDWR | O_NONBLOCK);
    if (ctx->uart_fd < 0)
    {
        syslog(LOG_ERR, "[telemetry] Failed to open %s: %d\n",
               CONFIG_UAV_TELEMETRY_UART_DEV, errno);
        ctx->is_running = false;
        return -1;
    }

    if (configure_uart(ctx->uart_fd, CONFIG_UAV_TELEMETRY_BAUDRATE) < 0)
    {
        syslog(LOG_ERR, "[telemetry] Failed to configure UART\n");
        close(ctx->uart_fd);
        ctx->uart_fd = -1;
        ctx->is_running = false;
        return -1;
    }

    syslog(LOG_INFO, "[telemetry] UART %s opened @ %d baud\n",
           CONFIG_UAV_TELEMETRY_UART_DEV, CONFIG_UAV_TELEMETRY_BAUDRATE);

    /*=========================================================================
     * PHASE 3: Subscribe to uORB topics
     *=========================================================================*/

    ctx->combined_sub = uorb::orb_subscribe(ORB_ID(sensor_combined));
    if (ctx->combined_sub < 0)
    {
        syslog(LOG_WARNING, "[telemetry] Failed to subscribe sensor_combined\n");
    }

    ctx->status_sub = uorb::orb_subscribe(ORB_ID(system_status));
    if (ctx->status_sub < 0)
    {
        syslog(LOG_WARNING, "[telemetry] Failed to subscribe system_status\n");
    }

    syslog(LOG_INFO, "[telemetry] uORB subscriptions ready\n");

    /*=========================================================================
     * PHASE 4: Main loop
     *=========================================================================*/

    const useconds_t period_us = 1000000 / CONFIG_UAV_TELEMETRY_RATE_HZ;

    syslog(LOG_INFO, "[telemetry] Starting main loop @ %d Hz\n",
           CONFIG_UAV_TELEMETRY_RATE_HZ);

    ctx->is_running = true;

    while (!ctx->should_exit)
    {
        uint64_t loop_start = hrt_absolute_time();

        /* Send telemetry packet */
        send_packet(ctx);

        /* Update loop counter */
        ctx->loop_count++;

        /* Sleep until next period */
        uint64_t elapsed = hrt_absolute_time() - loop_start;
        if (elapsed < period_us)
        {
            usleep(period_us - elapsed);
        }
    }

    /*=========================================================================
     * PHASE 5: Cleanup
     *=========================================================================*/

    syslog(LOG_INFO, "[telemetry] Shutting down...\n");

    if (ctx->combined_sub >= 0)
    {
        uorb::orb_unsubscribe(ctx->combined_sub);
        ctx->combined_sub = -1;
    }

    if (ctx->status_sub >= 0)
    {
        uorb::orb_unsubscribe(ctx->status_sub);
        ctx->status_sub = -1;
    }

    if (ctx->uart_fd >= 0)
    {
        close(ctx->uart_fd);
        ctx->uart_fd = -1;
    }

    ctx->is_running = false;

    syslog(LOG_INFO, "[telemetry] Thread exited. Sent %lu packets, %lu errors\n",
           (unsigned long)ctx->packets_sent,
           (unsigned long)ctx->send_errors);

    return 0;
}

/****************************************************************************
 * Command Handlers
 ****************************************************************************/

static void print_usage(void)
{
    printf("Usage: telemetry <command>\n");
    printf("Commands:\n");
    printf("  start   - Start telemetry\n");
    printf("  stop    - Stop telemetry\n");
    printf("  status  - Show status\n");
}

static int cmd_start(void)
{
    TelemetryContext *ctx = &g_telem_ctx;

    if (ctx->is_running)
    {
        printf("[telemetry] Already running\n");
        return -1;
    }

    /* Reset context */
    ctx->reset();

    /* Start task */
    ctx->task_pid = task_create("telemetry",
                                CONFIG_UAV_TELEMETRY_PRIORITY,
                                CONFIG_UAV_TELEMETRY_STACKSIZE,
                                telemetry_thread_main,
                                NULL);

    if (ctx->task_pid < 0)
    {
        printf("[telemetry] Failed to start task: %d\n", errno);
        return -1;
    }

    printf("[telemetry] Started (pid=%d)\n", ctx->task_pid);
    return 0;
}

static int cmd_stop(void)
{
    TelemetryContext *ctx = &g_telem_ctx;

    if (!ctx->is_running)
    {
        printf("[telemetry] Not running\n");
        return -1;
    }

    ctx->should_exit = true;

    /* Wait for thread to exit */
    for (int i = 0; i < 50; i++)
    {
        usleep(20000);
        if (!ctx->is_running)
        {
            break;
        }
    }

    if (ctx->is_running)
    {
        printf("[telemetry] Warning: Thread did not exit cleanly\n");
    }
    else
    {
        printf("[telemetry] Stopped\n");
    }

    return 0;
}

static int cmd_status(void)
{
    TelemetryContext *ctx = &g_telem_ctx;

    printf("=== TELEMETRY STATUS ===\n");
    printf("Running:       %s\n", ctx->is_running ? "YES" : "NO");
    printf("UART:          %s @ %d baud\n",
           CONFIG_UAV_TELEMETRY_UART_DEV,
           CONFIG_UAV_TELEMETRY_BAUDRATE);
    printf("Rate:          %d Hz\n", CONFIG_UAV_TELEMETRY_RATE_HZ);
    printf("Packet size:   %d bytes\n", TELEM_PACKET_SIZE);
    printf("\n");
    printf("Packets sent:  %lu\n", (unsigned long)ctx->packets_sent);
    printf("Send errors:   %lu\n", (unsigned long)ctx->send_errors);
    printf("Loop count:    %lu\n", (unsigned long)ctx->loop_count);
    printf("Sequence:      %u\n", ctx->sequence);
    printf("\n");
    printf("Throughput:    %.1f KB/s\n",
           (float)(ctx->packets_sent * TELEM_PACKET_SIZE) / 1024.0f /
           ((float)ctx->loop_count / CONFIG_UAV_TELEMETRY_RATE_HZ + 0.001f));

    return 0;
}

/****************************************************************************
 * Public Entry Point
 ****************************************************************************/

extern "C" int telemetry_main(int argc, char *argv[])
{
    if (argc < 2)
    {
        print_usage();
        return -1;
    }

    const char *cmd = argv[1];

    if (strcmp(cmd, "start") == 0)
    {
        return cmd_start();
    }
    else if (strcmp(cmd, "stop") == 0)
    {
        return cmd_stop();
    }
    else if (strcmp(cmd, "status") == 0)
    {
        return cmd_status();
    }
    else
    {
        printf("Unknown command: %s\n", cmd);
        print_usage();
        return -1;
    }
}
