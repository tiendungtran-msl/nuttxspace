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

/* uORB publication */
static orb_advert_t g_gps_pub = nullptr;

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

    /* Create GPS driver instance */
    g_gps_driver = new GPSUbx();
    if (!g_gps_driver)
    {
        printf("[GPS] Failed to allocate driver\n");
        g_gps_running = false;
        return nullptr;
    }

    /* Initialize GPS driver */
    int ret = g_gps_driver->init(GPS_UART_PATH);
    if (ret < 0)
    {
        printf("[GPS] Init failed: %d\n", ret);
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
        /* Don't fail - GPS might send default messages */
    }

    /* Advertise uORB topic */
    sensor_gps_s gps_msg;
    memset(&gps_msg, 0, sizeof(gps_msg));
    g_gps_pub = orb_advertise(ORB_ID(sensor_gps), &gps_msg);

    printf("[GPS] Driver ready, publishing to sensor_gps\n");

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
                    orb_publish(ORB_ID(sensor_gps), g_gps_pub, &gps_msg);
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
        orb_unadvertise(g_gps_pub);
        g_gps_pub = nullptr;
    }

    g_gps_driver->deinit();
    delete g_gps_driver;
    g_gps_driver = nullptr;

    g_gps_running = false;

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

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, GPS_STACK_SIZE);

    int ret = pthread_create(&g_gps_thread, &attr, gps_thread_main, nullptr);
    pthread_attr_destroy(&attr);

    if (ret != 0)
    {
        printf("[GPS] Failed to create thread: %d\n", ret);
        g_gps_running = false;
        return -ret;
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

    if (g_gps_driver)
    {
        printf("Configured: %s\n", g_gps_driver->isConfigured() ? "YES" : "NO");
        printf("Messages:   %u\n", g_gps_driver->getMessageCount());
        printf("Errors:     %u\n", g_gps_driver->getErrorCount());

        const gps_data_s *data = g_gps_driver->getData();
        printf("\nLast Position:\n");
        printf("  Fix Type:  %u\n", data->fix_type);
        printf("  Satellites: %u\n", data->num_sats);
        printf("  Latitude:  %.7f deg\n", data->lat);
        printf("  Longitude: %.7f deg\n", data->lon);
        printf("  Altitude:  %.2f m (MSL)\n", data->alt_msl);
        printf("  Speed N:   %.2f m/s\n", data->vel_n);
        printf("  Speed E:   %.2f m/s\n", data->vel_e);
        printf("  Speed D:   %.2f m/s\n", data->vel_d);
        printf("  H Acc:     %.2f m\n", data->hacc);
        printf("  V Acc:     %.2f m\n", data->vacc);
        printf("  S Acc:     %.2f m/s\n", data->sacc);
        printf("  PDOP:      %.2f\n", data->pdop);

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
 * @brief Print usage
 */
static void print_usage(void)
{
    printf("\nGPS Driver for u-blox M10N\n");
    printf("Usage: gps_app <command>\n\n");
    printf("Commands:\n");
    printf("  start   - Start GPS driver\n");
    printf("  stop    - Stop GPS driver\n");
    printf("  status  - Show GPS status\n");
    printf("  help    - Show this help\n\n");
    printf("UART: %s\n\n", GPS_UART_PATH);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C" int main(int argc, char *argv[]);

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        print_usage();
        return 1;
    }

    const char *cmd = argv[1];

    if (strcmp(cmd, "start") == 0)
    {
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
