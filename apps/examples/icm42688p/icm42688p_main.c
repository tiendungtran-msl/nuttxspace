/****************************************************************************
 * apps/examples/icm42688p/icm42688p_main.c
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include "icm42688p_driver.h"

static volatile bool g_running = true;

static void signal_handler(int signo)
{
    g_running = false;
}

static void print_usage(const char *progname)
{
    printf("Usage: %s [OPTIONS]\n", progname);
    printf("Options:\n");
    printf("  -c          Calibrate gyroscope\n");
    printf("  -n <count>  Number of samples to read (default: continuous)\n");
    printf("  -r <rate>   Sample rate in Hz (default: 100)\n");
    printf("  -h          Show this help\n");
}

int main(int argc, FAR char *argv[])
{
    icm42688p_dev_t dev;
    icm42688p_data_t data;
    bool do_calibration = false;
    int sample_count = -1;  /* -1 = continuous */
    int sample_rate_hz = 100;
    int opt;
    int ret;
    int samples_read = 0;

    /* Parse command line arguments */
    while ((opt = getopt(argc, argv, "cn:r:h")) != ERROR) {
        switch (opt) {
            case 'c':
                do_calibration = true;
                break;
            case 'n':
                sample_count = atoi(optarg);
                break;
            case 'r':
                sample_rate_hz = atoi(optarg);
                if (sample_rate_hz <= 0 || sample_rate_hz > 1000) {
                    fprintf(stderr, "Invalid sample rate\n");
                    return EXIT_FAILURE;
                }
                break;
            case 'h':
                print_usage(argv[0]);
                return EXIT_SUCCESS;
            default:
                print_usage(argv[0]);
                return EXIT_FAILURE;
        }
    }

    /* Setup signal handler for clean exit */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("\n=== ICM-42688-P NuttX Application ===\n\n");

    /* Initialize driver */
    ret = icm42688p_initialize(&dev);
    if (ret < 0) {
        fprintf(stderr, "ERROR: Failed to initialize ICM-42688-P: %d\n", ret);
        return EXIT_FAILURE;
    }

    /* Calibrate gyroscope if requested */
    if (do_calibration) {
        ret = icm42688p_calibrate_gyro(&dev, 200);
        if (ret < 0) {
            fprintf(stderr, "ERROR: Calibration failed: %d\n", ret);
            goto cleanup;
        }
    }

    printf("Starting data acquisition at %d Hz...\n", sample_rate_hz);
    printf("Press Ctrl+C to stop\n\n");
    printf("Time(s)  | Accel X   Y   Z (g)     | Gyro X     Y     Z (dps)    | Temp(°C)\n");
    printf("---------|-------------------------|-----------------------------|---------\n");

    /* Main data acquisition loop */
    while (g_running && (sample_count < 0 || samples_read < sample_count)) {
        ret = icm42688p_read_data(&dev, &data);
        if (ret < 0) {
            fprintf(stderr, "ERROR: Failed to read data: %d\n", ret);
            usleep(100000);
            continue;
        }

        /* Print data */
        printf("%7.3f  | %6.3f %6.3f %6.3f | %7.2f %7.2f %7.2f | %6.2f\n",
               data.timestamp / 1000000.0,
               data.accel.x, data.accel.y, data.accel.z,
               data.gyro.x, data.gyro.y, data.gyro.z,
               data.temperature);

        samples_read++;

        /* Sleep to maintain sample rate */
        usleep(1000000 / sample_rate_hz);
    }

    printf("\n");
    icm42688p_print_status(&dev);

cleanup:
    /* Cleanup */
    icm42688p_deinitialize(&dev);
    printf("Application terminated\n");

    return EXIT_SUCCESS;
}