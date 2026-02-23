/****************************************************************************
 * apps/uav/drivers/gps/gps_main.cpp
 *
 * GPS Driver Application for u-blox M10N
 *
 * Usage:
 *   gps_app start   - Start GPS driver
 *   gps_app stop    - Stop GPS driver
 *   gps_app status  - Show GPS status
 *
 * Publishes:
 *   - sensor_gps (uORB topic)
 *
 * UART Configuration:
 *   - UART4 on STM32H7: PA0(TX), PA1(RX)
 *   - Default baudrate: 9600 -> auto-switch to 115200
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/clock.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <math.h>
#include <limits.h>
#include <poll.h>
#include <termios.h>

#include "gps_ubx.hpp"
#include "../../uorb/uorb.hpp"
#include "../../uorb/topics/sensor_gps.hpp"

/* Include the gps_ubx implementation directly to avoid build system issues */
#include "gps_ubx.cpp"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPS UART device path - UART4 on STM32H7
 * Device numbering: USART1=ttyS0 (console), UART4=ttyS1, UART8=ttyS2
 */
#ifndef CONFIG_UAV_GPS_UART_PATH
#define GPS_UART_PATH       "/dev/ttyS1"
#else
#define GPS_UART_PATH       CONFIG_UAV_GPS_UART_PATH
#endif

/* GPS polling rate */
#define GPS_POLL_INTERVAL_MS    20      /* 50 Hz polling */

/* Thread stack size */
#define GPS_STACK_SIZE          2048

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile bool g_gps_running = false;
static volatile bool g_gps_should_stop = false;
static pthread_t g_gps_thread;
static GPSUbx *g_gps_driver = nullptr;
static int g_gps_last_error = 0;
static const char *g_gps_active_uart = nullptr;
static char g_gps_uart_override[32];

enum gps_init_state_e
{
    GPS_INIT_IDLE = 0,
    GPS_INIT_STARTING,
    GPS_INIT_READY,
    GPS_INIT_FAILED
};

static volatile int g_gps_init_state = GPS_INIT_IDLE;

/* uORB publication */
static uorb::orb_advert_t g_gps_pub = nullptr;

static const char *gps_init_state_str(int state)
{
    switch (state)
    {
        case GPS_INIT_IDLE:     return "IDLE";
        case GPS_INIT_STARTING: return "STARTING";
        case GPS_INIT_READY:    return "READY";
        case GPS_INIT_FAILED:   return "FAILED";
        default:                return "UNKNOWN";
    }
}

static int gps_uart_quick_probe(const char *uart_path, int baudrate,
                                int duration_ms, bool *has_data,
                                bool *has_ubx, bool *has_nmea,
                                int *bytes_read)
{
    *has_data = false;
    *has_ubx = false;
    *has_nmea = false;
    *bytes_read = 0;

    int fd = open(uart_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        return -errno;
    }

    struct termios tio;
    memset(&tio, 0, sizeof(tio));

    if (tcgetattr(fd, &tio) != 0)
    {
        int ret = -errno;
        close(fd);
        return ret;
    }

    speed_t speed = B9600;
    if (baudrate == 115200)
    {
        speed = B115200;
    }

    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag |= CLOCAL;
    tio.c_cflag |= CREAD;
    tio.c_cflag &= ~CRTSCTS;

    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    tio.c_oflag &= ~OPOST;

    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0)
    {
        int ret = -errno;
        close(fd);
        return ret;
    }

    tcflush(fd, TCIOFLUSH);

    uint8_t buf[128];
    int elapsed_ms = 0;

    while (elapsed_ms < duration_ms)
    {
        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLIN;

        int poll_ret = poll(&pfd, 1, 100);
        elapsed_ms += 100;

        if (poll_ret < 0)
        {
            int ret = -errno;
            close(fd);
            return ret;
        }

        if (poll_ret == 0 || !(pfd.revents & POLLIN))
        {
            continue;
        }

        ssize_t nread = read(fd, buf, sizeof(buf));
        if (nread < 0)
        {
            if (errno == EAGAIN)
            {
                continue;
            }

            int ret = -errno;
            close(fd);
            return ret;
        }

        if (nread == 0)
        {
            continue;
        }

        *has_data = true;
        *bytes_read += (int)nread;

        for (ssize_t i = 0; i < nread; i++)
        {
            if (buf[i] == '$')
            {
                *has_nmea = true;
            }

            if (i + 1 < nread && buf[i] == 0xB5 && buf[i + 1] == 0x62)
            {
                *has_ubx = true;
            }
        }

        if (*has_ubx || *has_nmea)
        {
            break;
        }
    }

    close(fd);
    return 0;
}

static int gps_selftest(void)
{
    if (g_gps_running)
    {
        printf("[GPS] selftest unavailable while driver is running\n");
        return -EBUSY;
    }

    const char *uart_candidates[] =
    {
        GPS_UART_PATH,
        "/dev/ttyS1",
        "/dev/ttyS2",
        "/dev/ttyS3"
    };

    printf("\n=== GPS Selftest ===\n");
    printf("Hint: ensure GPS is powered and TX from GPS is connected to MCU RX.\n");

    bool any_open_ok = false;
    bool any_signal = false;
    bool any_protocol = false;

    for (unsigned int i = 0; i < sizeof(uart_candidates) / sizeof(uart_candidates[0]); i++)
    {
        const char *uart_path = uart_candidates[i];
        bool duplicate = false;

        for (unsigned int j = 0; j < i; j++)
        {
            if (strcmp(uart_candidates[j], uart_path) == 0)
            {
                duplicate = true;
                break;
            }
        }

        if (duplicate)
        {
            continue;
        }

        for (unsigned int b = 0; b < 2; b++)
        {
            int baud = (b == 0) ? 9600 : 115200;
            bool has_data = false;
            bool has_ubx = false;
            bool has_nmea = false;
            int bytes = 0;

            int ret = gps_uart_quick_probe(uart_path, baud, 1200,
                                           &has_data, &has_ubx, &has_nmea, &bytes);
            if (ret < 0)
            {
                printf("[SELFTEST] %s @%d: open/probe failed (%d)\n", uart_path, baud, ret);
                continue;
            }

            any_open_ok = true;

            if (has_data)
            {
                any_signal = true;
            }

            if (has_ubx || has_nmea)
            {
                any_protocol = true;
            }

            printf("[SELFTEST] %s @%d: bytes=%d ubx=%s nmea=%s\n",
                   uart_path,
                   baud,
                   bytes,
                   has_ubx ? "YES" : "NO",
                   has_nmea ? "YES" : "NO");
        }
    }

    if (!any_open_ok)
    {
        printf("[SELFTEST] FAIL: cannot open any candidate UART device\n");
        printf("[SELFTEST] Check serial driver config and /dev/ttySx nodes\n");
        printf("====================\n\n");
        return -ENODEV;
    }

    if (!any_signal)
    {
        printf("[SELFTEST] FAIL: UART opens but no incoming bytes\n");
        printf("[SELFTEST] Likely causes: GPS not powered, TX/RX wiring mismatch, wrong port\n");
        printf("====================\n\n");
        return -EIO;
    }

    if (!any_protocol)
    {
        printf("[SELFTEST] WARN: bytes detected but UBX/NMEA signature not seen\n");
        printf("[SELFTEST] Check baudrate/protocol settings of GPS module\n");
        printf("====================\n\n");
        return -EAGAIN;
    }

    printf("[SELFTEST] PASS: GPS signal/protocol detected\n");
    printf("====================\n\n");
    return 0;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief GPS driver thread
 */
static void *gps_thread_main(void *arg)
{
    (void)arg;

    printf("[GPS] Thread started\n");
    g_gps_last_error = 0;
    g_gps_active_uart = nullptr;
    g_gps_init_state = GPS_INIT_STARTING;

    /* Create GPS driver instance */
    g_gps_driver = new GPSUbx();
    if (!g_gps_driver)
    {
        printf("[GPS] Failed to allocate driver\n");
        g_gps_last_error = -ENOMEM;
        g_gps_init_state = GPS_INIT_FAILED;
        g_gps_running = false;
        return nullptr;
    }

    /* Initialize GPS driver: probe likely UART device paths */
    int ret = -ENODEV;
    const char *uart_candidates[] =
    {
        (g_gps_uart_override[0] != '\0') ? g_gps_uart_override : GPS_UART_PATH,
        GPS_UART_PATH,
        "/dev/ttyS1",
        "/dev/ttyS2",
        "/dev/ttyS3"
    };

    for (unsigned int i = 0; i < sizeof(uart_candidates) / sizeof(uart_candidates[0]); i++)
    {
        const char *uart_path = uart_candidates[i];
        bool duplicate = false;

        for (unsigned int j = 0; j < i; j++)
        {
            if (strcmp(uart_candidates[j], uart_path) == 0)
            {
                duplicate = true;
                break;
            }
        }

        if (duplicate)
        {
            continue;
        }

        ret = g_gps_driver->init(uart_path);
        if (ret == 0)
        {
            g_gps_active_uart = uart_path;
            printf("[GPS] Using UART: %s\n", uart_path);
            break;
        }

        printf("[GPS] init(%s) failed: %d\n", uart_path, ret);
        g_gps_last_error = ret;
    }

    if (ret < 0)
    {
        printf("[GPS] Init failed on all UART paths (last=%d)\n", ret);
        g_gps_init_state = GPS_INIT_FAILED;
        delete g_gps_driver;
        g_gps_driver = nullptr;
        g_gps_running = false;
        return nullptr;
    }

    /* Configure GPS module */
    ret = g_gps_driver->configure();
    if (ret < 0)
    {
        printf("[GPS] Configure failed: %d (continuing anyway)\n", ret);
        g_gps_last_error = ret;
        /* Don't fail - GPS might send default messages */
    }

    /* Advertise uORB topic */
    sensor_gps_s gps_msg;
    memset(&gps_msg, 0, sizeof(gps_msg));
    g_gps_pub = uorb::orb_advertise(ORB_ID(sensor_gps), &gps_msg);

    printf("[GPS] Driver ready, publishing to sensor_gps\n");
    g_gps_init_state = GPS_INIT_READY;

    uint32_t loop_count = 0;
    uint32_t msg_count = 0;
    uint64_t last_print_time = 0;

    /* Main loop */
    while (!g_gps_should_stop)
    {
        /* Poll for GPS data */
        ret = g_gps_driver->poll(GPS_POLL_INTERVAL_MS);

        if (ret > 0)
        {
            /* New data available */
            const gps_data_s *data = g_gps_driver->getData();

            if (data->position_valid)
            {
                /* Fill uORB message */
                gps_msg.timestamp_us = data->timestamp_us;
                gps_msg.lat = data->lat;
                gps_msg.lon = data->lon;
                gps_msg.alt = data->alt_msl;
                gps_msg.vel_n = data->vel_n;
                gps_msg.vel_e = data->vel_e;
                gps_msg.vel_d = data->vel_d;
                gps_msg.hacc = data->hacc;
                gps_msg.vacc = data->vacc;
                gps_msg.sacc = data->sacc;
                gps_msg.fix_type = data->fix_type;
                gps_msg.nsats = data->num_sats;

                /* Publish to uORB */
                if (g_gps_pub)
                {
                    uorb::orb_publish(ORB_ID(sensor_gps), g_gps_pub, &gps_msg);
                }

                msg_count++;
            }
        }

        loop_count++;

        /* Print status every 5 seconds */
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;

        if (now - last_print_time > 5000000ULL)  /* 5 seconds */
        {
            const gps_data_s *data = g_gps_driver->getData();
            printf("[GPS] fix=%u sats=%u lat=%.6f lon=%.6f alt=%.1f msgs=%u errs=%u\n",
                   data->fix_type,
                   data->num_sats,
                   data->lat,
                   data->lon,
                   data->alt_msl,
                   g_gps_driver->getMessageCount(),
                   g_gps_driver->getErrorCount());
            last_print_time = now;
        }
    }

    printf("[GPS] Thread stopping (loops=%u, msgs=%u)\n", loop_count, msg_count);

    /* Cleanup */
    if (g_gps_pub)
    {
        uorb::orb_unadvertise(g_gps_pub);
        g_gps_pub = nullptr;
    }

    g_gps_driver->deinit();
    delete g_gps_driver;
    g_gps_driver = nullptr;

    g_gps_running = false;
    g_gps_init_state = GPS_INIT_IDLE;

    return nullptr;
}

/**
 * @brief Start GPS driver
 */
static int gps_start(void)
{
    if (g_gps_running)
    {
        printf("[GPS] Already running\n");
        return -EBUSY;
    }

    g_gps_should_stop = false;
    g_gps_running = true;
    g_gps_init_state = GPS_INIT_STARTING;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, GPS_STACK_SIZE);

    int ret = pthread_create(&g_gps_thread, &attr, gps_thread_main, nullptr);
    pthread_attr_destroy(&attr);

    if (ret != 0)
    {
        printf("[GPS] Failed to create thread: %d\n", ret);
        g_gps_last_error = -ret;
        g_gps_init_state = GPS_INIT_FAILED;
        g_gps_running = false;
        return -ret;
    }

    for (int i = 0; i < 40; i++)
    {
        if (g_gps_init_state == GPS_INIT_READY || g_gps_init_state == GPS_INIT_FAILED)
        {
            break;
        }

        usleep(50000); /* 50 ms */
    }

    if (g_gps_init_state == GPS_INIT_FAILED)
    {
        printf("[GPS] Start failed (last_error=%d)\n", g_gps_last_error);
        return g_gps_last_error ? g_gps_last_error : -EIO;
    }

    if (g_gps_init_state != GPS_INIT_READY)
    {
        printf("[GPS] Start pending (state=%s)\n", gps_init_state_str(g_gps_init_state));
    }

    printf("[GPS] Started\n");
    return 0;
}

/**
 * @brief Stop GPS driver
 */
static int gps_stop(void)
{
    if (!g_gps_running)
    {
        printf("[GPS] Not running\n");
        return -ESRCH;
    }

    g_gps_should_stop = true;

    /* Wait for thread to finish */
    int ret = pthread_join(g_gps_thread, nullptr);
    if (ret != 0)
    {
        printf("[GPS] Failed to join thread: %d\n", ret);
    }

    printf("[GPS] Stopped\n");
    return 0;
}

/**
 * @brief Print GPS status
 */
static void gps_status(void)
{
    printf("\n=== GPS Driver Status ===\n");
    printf("Running: %s\n", g_gps_running ? "YES" : "NO");
    printf("Init state: %s\n", gps_init_state_str(g_gps_init_state));
    printf("Active UART: %s\n", g_gps_active_uart ? g_gps_active_uart : "none");
    printf("Last error: %d\n", g_gps_last_error);

    if (g_gps_driver)
    {
        printf("Configured: %s\n", g_gps_driver->isConfigured() ? "YES" : "NO");
        printf("Messages:   %u\n", g_gps_driver->getMessageCount());
        printf("Errors:     %u\n", g_gps_driver->getErrorCount());

        const gps_data_s *data = g_gps_driver->getData();
        printf("Position valid: %s\n", data->position_valid ? "YES" : "NO");
        printf("Velocity valid: %s\n", data->velocity_valid ? "YES" : "NO");
        printf("\nLast Position:\n");
        printf("  Fix Type:  %u\n", data->fix_type);
        printf("  Satellites: %u\n", data->num_sats);

        if (!data->position_valid)
        {
            printf("  No valid GNSS fix yet (waiting for satellites / sky view).\n");
        }
        else
        {
            printf("  Latitude:  %.7f deg\n", data->lat);
            printf("  Longitude: %.7f deg\n", data->lon);
            printf("  Altitude:  %.2f m (MSL)\n", data->alt_msl);
            printf("  Speed N:   %.2f m/s\n", data->vel_n);
            printf("  Speed E:   %.2f m/s\n", data->vel_e);
            printf("  Speed D:   %.2f m/s\n", data->vel_d);

            if (isfinite(data->hacc))
            {
                printf("  H Acc:     %.2f m\n", data->hacc);
            }

            if (isfinite(data->vacc))
            {
                printf("  V Acc:     %.2f m\n", data->vacc);
            }

            if (isfinite(data->sacc))
            {
                printf("  S Acc:     %.2f m/s\n", data->sacc);
            }

            if (isfinite(data->pdop))
            {
                printf("  PDOP:      %.2f\n", data->pdop);
            }
        }

        if (data->time_valid)
        {
            printf("  Time (UTC): %04u-%02u-%02u %02u:%02u:%02u\n",
                   data->year, data->month, data->day,
                   data->hour, data->minute, data->second);
        }
    }
    printf("========================\n\n");
}

/**
 * @brief Print current GPS information (compact)
 */
static void gps_info(void)
{
    if (!g_gps_running || !g_gps_driver)
    {
        printf("[GPS] Driver not running (last_error=%d, uart=%s)\n",
               g_gps_last_error,
               g_gps_active_uart ? g_gps_active_uart : "none");
        return;
    }

    const gps_data_s *data = g_gps_driver->getData();

    printf("\n=== Current GPS Info ===\n");
    printf("Fix:        %u\n", data->fix_type);
    printf("Satellites: %u\n", data->num_sats);
    printf("Pos valid:  %s\n", data->position_valid ? "YES" : "NO");

    if (data->position_valid)
    {
        printf("Latitude:   %.7f deg\n", data->lat);
        printf("Longitude:  %.7f deg\n", data->lon);
        printf("Altitude:   %.2f m\n", data->alt_msl);

        float ground_speed = sqrtf(data->vel_n * data->vel_n + data->vel_e * data->vel_e);
        printf("Speed:      %.2f m/s\n", ground_speed);
    }
    else
    {
        printf("Position:   waiting for valid fix\n");
    }

    if (data->time_valid)
    {
        printf("UTC Time:   %04u-%02u-%02u %02u:%02u:%02u\n",
               data->year, data->month, data->day,
               data->hour, data->minute, data->second);
    }
    else
    {
        printf("UTC Time:   invalid\n");
    }

    printf("========================\n\n");
}

/**
 * @brief Print usage
 */
static void print_usage(void)
{
    printf("\nGPS Driver for u-blox M10N\n");
    printf("Usage: gps_app <command> [uart_dev]\n\n");
    printf("Commands:\n");
    printf("  start [dev] - Start GPS driver (optional dev: /dev/ttyS1, /dev/ttyS2, ...)\n");
    printf("  stop    - Stop GPS driver\n");
    printf("  status  - Show GPS status\n");
    printf("  info    - Show current GPS information\n");
    printf("  selftest - Probe UART ports and detect UBX/NMEA data\n");
    printf("  help    - Show this help\n\n");
    printf("UART: %s\n\n", GPS_UART_PATH);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C" int gps_app_main(int argc, char *argv[]);

int gps_app_main(int argc, char *argv[])
{
    if (argc < 2)
    {
        print_usage();
        return 1;
    }

    const char *cmd = argv[1];

    if (strcmp(cmd, "start") == 0)
    {
        g_gps_uart_override[0] = '\0';

        if (argc >= 3)
        {
            strncpy(g_gps_uart_override, argv[2], sizeof(g_gps_uart_override) - 1);
            g_gps_uart_override[sizeof(g_gps_uart_override) - 1] = '\0';
            printf("[GPS] UART override requested: %s\n", g_gps_uart_override);
        }

        return gps_start();
    }
    else if (strcmp(cmd, "stop") == 0)
    {
        return gps_stop();
    }
    else if (strcmp(cmd, "status") == 0)
    {
        gps_status();
        return 0;
    }
    else if (strcmp(cmd, "info") == 0)
    {
        gps_info();
        return 0;
    }
    else if (strcmp(cmd, "selftest") == 0)
    {
        return gps_selftest();
    }
    else if (strcmp(cmd, "help") == 0)
    {
        print_usage();
        return 0;
    }
    else
    {
        printf("Unknown command: %s\n", cmd);
        print_usage();
        return 1;
    }
}
