/****************************************************************************
 * apps/uav/telemetry_app/telemetry_main.cpp
 *
 * UAV TELEMETRY APPLICATION
 *
 * MỤC ĐÍCH:
 * - Subscribe các uORB topic cần thiết
 * - Đóng gói dữ liệu vào binary packet 212 bytes
 * - Gửi dữ liệu 4 IMU riêng lẻ (raw từ chip)
 * - Gửi qua UART với DMA (non-blocking)
 * - Chạy ở priority thấp, không ảnh hưởng realtime tasks
 *
 * TIMING:
 * - Rate: 100 Hz (10ms period)
 * - UART: 921600 baud
 * - Packet: 212 bytes → ~2.3ms transfer time
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
    int imu_sub[TELEM_NUM_IMUS];    /* 4 individual IMU subscriptions */

    /* Cached sensor data - giữ giá trị cuối cùng */
    sensor_combined_s cached_combined;
    vehicle_attitude_s cached_attitude;
    sensor_imu_s cached_imu[TELEM_NUM_IMUS];    /* 4 IMU data */
    system_status_s cached_status;

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
    int last_send_errno;
    int last_send_written;
    uint32_t loop_count;
    uint64_t last_send_time;

    /* Constructor */
    TelemetryContext() :
        uart_fd(-1),
        combined_sub(-1),
        status_sub(-1),
        attitude_sub(-1),
        sequence(0),
        should_exit(false),
        is_running(false),
        task_pid(-1),
        packets_sent(0),
        send_errors(0),
        last_send_errno(0),
        last_send_written(0),
        loop_count(0),
        last_send_time(0)
    {
        memset(&packet, 0, sizeof(packet));
        memset(&cached_combined, 0, sizeof(cached_combined));
        memset(&cached_attitude, 0, sizeof(cached_attitude));
        memset(&cached_status, 0, sizeof(cached_status));
        for (int i = 0; i < TELEM_NUM_IMUS; i++)
        {
            imu_sub[i] = -1;
            memset(&cached_imu[i], 0, sizeof(sensor_imu_s));
        }
    }

    void reset()
    {
        uart_fd = -1;
        combined_sub = -1;
        status_sub = -1;
        attitude_sub = -1;
        for (int i = 0; i < TELEM_NUM_IMUS; i++)
        {
            imu_sub[i] = -1;
        }
        sequence = 0;
        should_exit = false;
        is_running = false;
        task_pid = -1;
        packets_sent = 0;
        send_errors = 0;
        last_send_errno = 0;
        last_send_written = 0;
        loop_count = 0;
        last_send_time = 0;
        memset(&packet, 0, sizeof(packet));
    }
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static TelemetryContext g_telem_ctx;

/* Runtime configurable UART device - mặc định từ config */
static char g_uart_device[32] = CONFIG_UAV_TELEMETRY_UART_DEV;

/* Runtime configurable UART baudrate */
static int g_uart_baud = CONFIG_UAV_TELEMETRY_BAUDRATE;

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
     * IMU DATA - đọc 4 IMU riêng lẻ từ sensor_imu topics
     * Gửi dữ liệu raw từ chip, không phải fused
     *=========================================================================*/

    for (int i = 0; i < TELEM_NUM_IMUS; i++)
    {
        if (ctx->imu_sub[i] >= 0)
        {
            bool updated = false;
            uorb::orb_check(ctx->imu_sub[i], &updated);

            if (updated)
            {
                uorb::orb_copy(ORB_ID(sensor_imu), ctx->imu_sub[i], &ctx->cached_imu[i]);
            }
        }

        /* Luôn sử dụng cached data cho mỗi IMU */
        pkt->imu[i].gyro_x = ctx->cached_imu[i].gyro[0];
        pkt->imu[i].gyro_y = ctx->cached_imu[i].gyro[1];
        pkt->imu[i].gyro_z = ctx->cached_imu[i].gyro[2];
        pkt->imu[i].accel_x = ctx->cached_imu[i].accel[0];
        pkt->imu[i].accel_y = ctx->cached_imu[i].accel[1];
        pkt->imu[i].accel_z = ctx->cached_imu[i].accel[2];
        pkt->imu[i].temperature = ctx->cached_imu[i].temperature;
    }

    /* Cũng đọc combined để có fused data cho attitude */
    if (ctx->combined_sub >= 0)
    {
        bool updated = false;
        uorb::orb_check(ctx->combined_sub, &updated);

        if (updated)
        {
            uorb::orb_copy(ORB_ID(sensor_combined), ctx->combined_sub, &ctx->cached_combined);
        }
    }

    /*=========================================================================
     * ATTITUDE DATA - từ vehicle_attitude topic
     * Luôn cập nhật cache nếu có data mới, sau đó dùng cache
     *=========================================================================*/

    if (ctx->attitude_sub >= 0)
    {
        bool updated = false;
        uorb::orb_check(ctx->attitude_sub, &updated);

        if (updated)
        {
            uorb::orb_copy(ORB_ID(vehicle_attitude), ctx->attitude_sub, &ctx->cached_attitude);
        }
    }

    /* Luôn sử dụng cached attitude data */
    pkt->attitude.qw = ctx->cached_attitude.q[0];
    pkt->attitude.qx = ctx->cached_attitude.q[1];
    pkt->attitude.qy = ctx->cached_attitude.q[2];
    pkt->attitude.qz = ctx->cached_attitude.q[3];
    pkt->attitude.roll = ctx->cached_attitude.roll;
    pkt->attitude.pitch = ctx->cached_attitude.pitch;
    pkt->attitude.yaw = ctx->cached_attitude.yaw;
    pkt->attitude.innovation_var = 0.0f;

    /*=========================================================================
     * SYSTEM STATUS
     * Luôn cập nhật cache nếu có data mới, sau đó dùng cache
     *=========================================================================*/

    if (ctx->status_sub >= 0)
    {
        bool updated = false;
        uorb::orb_check(ctx->status_sub, &updated);

        if (updated)
        {
            uorb::orb_copy(ORB_ID(system_status), ctx->status_sub, &ctx->cached_status);
        }
    }

    /* Luôn sử dụng cached status data */
    pkt->status.health_level = ctx->cached_status.health_level;
    pkt->status.healthy_imus = ctx->cached_status.healthy_imus;

    /* Build sensor flags */
    uint8_t flags = 0;
    if (ctx->cached_status.healthy_imus > 0) flags |= SENSOR_IMU_OK;
    if (ctx->cached_status.baro_ok)          flags |= SENSOR_BARO_OK;
    if (ctx->cached_status.mag_ok)           flags |= SENSOR_MAG_OK;
    if (ctx->cached_status.gps_ok)           flags |= SENSOR_GPS_OK;
    if (ctx->cached_status.ekf_ok)           flags |= SENSOR_EKF_OK;
    if (ctx->cached_status.timebase_ok)      flags |= SENSOR_TIMEBASE_OK;
    pkt->status.sensor_flags = flags;

    /* CPU load: deadline_misses as proxy for now */
    pkt->status.cpu_load = (uint16_t)(ctx->cached_status.deadline_misses & 0xFFFF);

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

    /* Send over UART (handle short writes / non-blocking) */
    const uint8_t *buf = (const uint8_t *)&ctx->packet;
    size_t total = 0;
    int last_errno = 0;
    uint64_t start_us = hrt_absolute_time();
    const uint64_t timeout_us = 2000; /* keep well below 10ms loop period */

    while (total < TELEM_PACKET_SIZE)
    {
        ssize_t n = write(ctx->uart_fd, buf + total, TELEM_PACKET_SIZE - total);
        if (n > 0)
        {
            total += (size_t)n;
            continue;
        }

        if (n == 0)
        {
            usleep(100);
        }
        else
        {
            last_errno = errno;
            if (last_errno == EINTR)
            {
                continue;
            }
            if (last_errno == EAGAIN || last_errno == EWOULDBLOCK)
            {
                usleep(100);
            }
            else
            {
                break;
            }
        }

        if ((hrt_absolute_time() - start_us) > timeout_us)
        {
            if (last_errno == 0)
            {
                last_errno = ETIMEDOUT;
            }
            break;
        }
    }

    if (total != TELEM_PACKET_SIZE)
    {
        ctx->send_errors++;
        ctx->last_send_errno = last_errno;
        ctx->last_send_written = (int)total;
        syslog(LOG_WARNING, "[telemetry] UART short write: wrote %d/%d (errno=%d)\n",
               (int)total, (int)TELEM_PACKET_SIZE, last_errno);
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

    ctx->uart_fd = open(g_uart_device, O_RDWR | O_NONBLOCK);
    if (ctx->uart_fd < 0)
    {
        syslog(LOG_ERR, "[telemetry] Failed to open %s: %d\n",
               g_uart_device, errno);
        ctx->is_running = false;
        return -1;
    }

    if (configure_uart(ctx->uart_fd, g_uart_baud) < 0)
    {
        syslog(LOG_ERR, "[telemetry] Failed to configure UART\n");
        close(ctx->uart_fd);
        ctx->uart_fd = -1;
        ctx->is_running = false;
        return -1;
    }

        syslog(LOG_INFO, "[telemetry] UART %s opened @ %d baud\n",
            g_uart_device, g_uart_baud);

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

    ctx->attitude_sub = uorb::orb_subscribe(ORB_ID(vehicle_attitude));
    if (ctx->attitude_sub < 0)
    {
        syslog(LOG_WARNING, "[telemetry] Failed to subscribe vehicle_attitude\n");
    }

    /* Subscribe to 4 individual IMU topics */
    for (int i = 0; i < TELEM_NUM_IMUS; i++)
    {
        ctx->imu_sub[i] = uorb::orb_subscribe_multi(ORB_ID(sensor_imu), i);
        if (ctx->imu_sub[i] < 0)
        {
            syslog(LOG_WARNING, "[telemetry] Failed to subscribe sensor_imu[%d]\n", i);
        }
        else
        {
            syslog(LOG_INFO, "[telemetry] Subscribed to sensor_imu[%d]\n", i);
        }
    }

    syslog(LOG_INFO, "[telemetry] uORB subscriptions ready (4 IMUs)\n");

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

    if (ctx->attitude_sub >= 0)
    {
        uorb::orb_unsubscribe(ctx->attitude_sub);
        ctx->attitude_sub = -1;
    }

    for (int i = 0; i < TELEM_NUM_IMUS; i++)
    {
        if (ctx->imu_sub[i] >= 0)
        {
            uorb::orb_unsubscribe(ctx->imu_sub[i]);
            ctx->imu_sub[i] = -1;
        }
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
    printf("Usage: telemetry <command> [args]\n");
    printf("Commands:\n");
    printf("  start            - Start telemetry\n");
    printf("  stop             - Stop telemetry\n");
    printf("  status           - Show status\n");
    printf("  port <device>    - Set UART device (e.g., /dev/ttyS0)\n");
    printf("  baud <rate>      - Set UART baudrate (115200/230400/460800/921600)\n");
    printf("  restart          - Restart telemetry with current config\n");
    printf("  scan             - Probe /dev/ttyS0..ttyS9 openability\n");
    printf("  test <dev> [baud] [sec] [mode] - Send test pattern to UART\n");
    printf("                     mode: packet (default) | ascii\n");
}

static bool is_supported_baud(int baud)
{
    return baud == 115200 || baud == 230400 || baud == 460800 || baud == 921600;
}

static int cmd_restart(void);  /* Forward declaration */

static int cmd_set_baud(const char *baud_str)
{
    TelemetryContext *ctx = &g_telem_ctx;

    if (!baud_str || strlen(baud_str) == 0)
    {
        printf("[telemetry] Error: Baudrate required\n");
        printf("Usage: telemetry baud <rate>\n");
        return -1;
    }

    int baud = atoi(baud_str);
    if (!is_supported_baud(baud))
    {
        printf("[telemetry] Error: Unsupported baud %d\n", baud);
        printf("Supported: 115200 230400 460800 921600\n");
        return -1;
    }

    g_uart_baud = baud;
    printf("[telemetry] Baud set to %d\n", g_uart_baud);

    if (ctx->is_running)
    {
        printf("[telemetry] Restarting to apply baud...\n");
        return cmd_restart();
    }

    return 0;
}

static int cmd_scan(void)
{
    printf("[telemetry] Probing serial devices (/dev/ttyS0..ttyS9)\n");
    for (int i = 0; i <= 9; i++)
    {
        char dev[32];
        snprintf(dev, sizeof(dev), "/dev/ttyS%d", i);

        int fd = open(dev, O_RDWR | O_NONBLOCK);
        if (fd < 0)
        {
            printf("  %s: FAIL (errno=%d)\n", dev, errno);
            continue;
        }

        close(fd);
        printf("  %s: OK\n", dev);
    }

    printf("[telemetry] Tip: Use 'telemetry test /dev/ttySx ...' then watch USB-TTL TX LED / scope\n");
    return 0;
}

static void fill_test_packet(struct telemetry_packet_s *pkt, uint16_t seq, uint32_t timestamp_us)
{
    telem_packet_init(pkt);

    /* 4 IMUs with obvious distinct values to spot on PC side */
    for (int i = 0; i < TELEM_NUM_IMUS; i++)
    {
        pkt->imu[i].gyro_x = 0.001f * (float)(seq & 0xFF) + (float)i;
        pkt->imu[i].gyro_y = 0.002f * (float)(seq & 0xFF) + (float)i;
        pkt->imu[i].gyro_z = 0.003f * (float)(seq & 0xFF) + (float)i;
        pkt->imu[i].accel_x = 0.1f + (float)i;
        pkt->imu[i].accel_y = 0.2f + (float)i;
        pkt->imu[i].accel_z = 9.81f + (float)i;
        pkt->imu[i].temperature = 25.0f + (float)i;
    }

    pkt->mag.mag_x = 0.5f;
    pkt->mag.mag_y = 0.0f;
    pkt->mag.mag_z = 0.0f;

    pkt->baro.pressure = 101325.0f;
    pkt->baro.altitude = 100.0f;

    pkt->gps.latitude = 210285110;     /* 21.0285110 deg * 1e7 */
    pkt->gps.longitude = 1058048170;   /* 105.8048170 deg * 1e7 */
    pkt->gps.altitude_msl = 100000;    /* 100 m in mm */
    pkt->gps.ground_speed = 500;       /* 5 m/s in cm/s */
    pkt->gps.heading = 9000;           /* 90 deg * 100 */
    pkt->gps.fix_type = GPS_FIX_3D;
    pkt->gps.satellites = 12;
    pkt->gps.hdop = 100;
    pkt->gps.vdop = 150;

    pkt->attitude.qw = 1.0f;
    pkt->attitude.qx = 0.0f;
    pkt->attitude.qy = 0.0f;
    pkt->attitude.qz = 0.0f;
    pkt->attitude.roll = 0.0f;
    pkt->attitude.pitch = 0.0f;
    pkt->attitude.yaw = 0.0f;
    pkt->attitude.innovation_var = 0.01f;

    pkt->status.health_level = HEALTH_GOOD;
    pkt->status.healthy_imus = 0x0F;
    pkt->status.sensor_flags = SENSOR_IMU_OK | SENSOR_TIMEBASE_OK;
    pkt->status.cpu_load = 100;
    pkt->status.battery_mv = 12600;
    pkt->status.loop_count = seq;

    telem_packet_finalize(pkt, seq, timestamp_us);
}

static int cmd_test(const char *device, int baudrate, int seconds, const char *mode)
{
    TelemetryContext *ctx = &g_telem_ctx;

    if (!device || strlen(device) == 0)
    {
        printf("[telemetry] Error: Device path required\n");
        printf("Usage: telemetry test <dev> [baud] [sec] [mode]\n");
        return -1;
    }

    if (ctx->is_running)
    {
        printf("[telemetry] Error: Stop telemetry before running test (telemetry stop)\n");
        return -1;
    }

    if (baudrate <= 0)
    {
        baudrate = CONFIG_UAV_TELEMETRY_BAUDRATE;
    }
    if (seconds <= 0)
    {
        seconds = 2;
    }
    if (!mode || strlen(mode) == 0)
    {
        mode = "packet";
    }

    int fd = open(device, O_RDWR | O_NONBLOCK);
    if (fd < 0)
    {
        printf("[telemetry] Error: Cannot open %s: %d\n", device, errno);
        return -1;
    }

    if (configure_uart(fd, baudrate) < 0)
    {
        printf("[telemetry] Error: configure_uart failed for %s (errno=%d)\n", device, errno);
        close(fd);
        return -1;
    }

    printf("[telemetry] TEST: device=%s baud=%d sec=%d mode=%s\n", device, baudrate, seconds, mode);
    printf("[telemetry] Watch USB-TTL TX LED / oscilloscope now...\n");

    uint32_t ok = 0;
    uint32_t err = 0;
    uint16_t seq = 0;
    uint64_t end_us = hrt_absolute_time() + (uint64_t)seconds * 1000000ULL;

    if (strcmp(mode, "ascii") == 0)
    {
        while (hrt_absolute_time() < end_us)
        {
            char line[64];
            int n = snprintf(line, sizeof(line), "TELEM ASCII TEST seq=%u\r\n", (unsigned)seq);
            ssize_t written = write(fd, line, n);
            if (written == n)
            {
                ok++;
            }
            else
            {
                err++;
            }
            seq++;
            usleep(10000);
        }
    }
    else
    {
        struct telemetry_packet_s pkt;
        while (hrt_absolute_time() < end_us)
        {
            uint64_t now_us = hrt_absolute_time();
            fill_test_packet(&pkt, seq, (uint32_t)now_us);
            ssize_t written = write(fd, &pkt, TELEM_PACKET_SIZE);
            if (written == TELEM_PACKET_SIZE)
            {
                ok++;
            }
            else
            {
                err++;
            }
            seq++;
            usleep(10000);
        }
    }

    close(fd);
    printf("[telemetry] TEST done: ok=%lu err=%lu\n", (unsigned long)ok, (unsigned long)err);
    return (err > 0) ? -1 : 0;
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
           g_uart_device,
            g_uart_baud);
    printf("Rate:          %d Hz\n", CONFIG_UAV_TELEMETRY_RATE_HZ);
    printf("Packet size:   %d bytes\n", TELEM_PACKET_SIZE);
    printf("\n");
    printf("Packets sent:  %lu\n", (unsigned long)ctx->packets_sent);
    printf("Send errors:   %lu\n", (unsigned long)ctx->send_errors);
    if (ctx->send_errors > 0)
    {
        printf("Last send err: errno=%d wrote=%d/%d\n",
               ctx->last_send_errno,
               ctx->last_send_written,
               (int)TELEM_PACKET_SIZE);
    }
    printf("Loop count:    %lu\n", (unsigned long)ctx->loop_count);
    printf("Sequence:      %u\n", ctx->sequence);
    printf("\n");
    printf("Throughput:    %.1f KB/s\n",
           (float)(ctx->packets_sent * TELEM_PACKET_SIZE) / 1024.0f /
           ((float)ctx->loop_count / CONFIG_UAV_TELEMETRY_RATE_HZ + 0.001f));

    return 0;
}

static int cmd_set_port(const char *device)
{
    TelemetryContext *ctx = &g_telem_ctx;

    if (!device || strlen(device) == 0)
    {
        printf("[telemetry] Error: Device path required\n");
        printf("Usage: telemetry port <device>\n");
        printf("Example: telemetry port /dev/ttyS0\n");
        return -1;
    }

    if (strlen(device) >= sizeof(g_uart_device))
    {
        printf("[telemetry] Error: Device path too long\n");
        return -1;
    }

    /* Kiểm tra xem cổng có tồn tại không */
    int test_fd = open(device, O_RDWR | O_NONBLOCK);
    if (test_fd < 0)
    {
        printf("[telemetry] Error: Cannot open %s: %d\n", device, errno);
        printf("Available ports: ls /dev/ttyS*\n");
        return -1;
    }
    close(test_fd);

    /* Lưu cấu hình mới */
    strncpy(g_uart_device, device, sizeof(g_uart_device) - 1);
    g_uart_device[sizeof(g_uart_device) - 1] = '\0';

    printf("[telemetry] UART device set to: %s\n", g_uart_device);

    /* Nếu đang chạy, cần restart */
    if (ctx->is_running)
    {
        printf("[telemetry] Restarting with new port...\n");
        cmd_stop();
        usleep(100000);  /* Wait 100ms */
        return cmd_start();
    }

    printf("[telemetry] Port will be used on next start\n");
    return 0;
}

static int cmd_restart(void)
{
    printf("[telemetry] Restarting...\n");
    cmd_stop();
    usleep(100000);  /* Wait 100ms */
    return cmd_start();
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
    else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "-h") == 0 || strcmp(cmd, "--help") == 0)
    {
        print_usage();
        return 0;
    }
    else if (strcmp(cmd, "stop") == 0)
    {
        return cmd_stop();
    }
    else if (strcmp(cmd, "status") == 0)
    {
        return cmd_status();
    }
    else if (strcmp(cmd, "port") == 0)
    {
        if (argc < 3)
        {
            printf("[telemetry] Error: Device path required\n");
            printf("Usage: telemetry port <device>\n");
            printf("Example: telemetry port /dev/ttyS0\n");
            return -1;
        }
        return cmd_set_port(argv[2]);
    }
    else if (strcmp(cmd, "baud") == 0)
    {
        if (argc < 3)
        {
            printf("[telemetry] Error: Baudrate required\n");
            printf("Usage: telemetry baud <rate>\n");
            return -1;
        }
        return cmd_set_baud(argv[2]);
    }
    else if (strcmp(cmd, "restart") == 0)
    {
        return cmd_restart();
    }
    else if (strcmp(cmd, "scan") == 0)
    {
        return cmd_scan();
    }
    else if (strcmp(cmd, "test") == 0)
    {
        if (argc < 3)
        {
            printf("[telemetry] Error: Device path required\n");
            printf("Usage: telemetry test <dev> [baud] [sec] [mode]\n");
            return -1;
        }

        int baud = (argc >= 4) ? atoi(argv[3]) : CONFIG_UAV_TELEMETRY_BAUDRATE;
        int sec = (argc >= 5) ? atoi(argv[4]) : 2;
        const char *mode = (argc >= 6) ? argv[5] : "packet";
        return cmd_test(argv[2], baud, sec, mode);
    }
    else
    {
        printf("Unknown command: %s\n", cmd);
        print_usage();
        return -1;
    }
}
