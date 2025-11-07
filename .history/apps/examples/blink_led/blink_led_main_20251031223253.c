/****************************************************************************
 * apps/examples/blink_led/blink_led_main.c
 *
 * Description:
 *   Blink LED application using GPIO device driver
 *   Toggles LED every 1 second (500ms ON, 500ms OFF)
 *
 * Author: tiendungtran-msl
 * Date: 2025-10-31
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>

#include <nuttx/ioexpander/gpio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_LED_DEVPATH "/dev/gpio0"
#define BLINK_PERIOD_MS  1000     /* 1 second period */
#define LED_ON_TIME_MS   500      /* 500ms ON */
#define LED_OFF_TIME_MS  500      /* 500ms OFF */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_blink_running = true;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: signal_handler
 *
 * Description:
 *   Handle SIGINT (Ctrl+C) to gracefully stop the application
 *
 ****************************************************************************/

static void signal_handler(int signo)
{
  if (signo == SIGINT)
    {
      printf("\nReceived SIGINT, stopping LED blink...\n");
      g_blink_running = false;
    }
}

/****************************************************************************
 * Name: print_usage
 *
 * Description:
 *   Print usage information
 *
 ****************************************************************************/

static void print_usage(FAR const char *progname)
{
  printf("\nUsage: %s [OPTIONS]\n", progname);
  printf("\nOptions:\n");
  printf("  -d <device>   GPIO device path (default: %s)\n", 
         GPIO_LED_DEVPATH);
  printf("  -t <seconds>  Total run time in seconds (default: run forever)\n");
  printf("  -p <ms>       Blink period in milliseconds (default: %d)\n",
         BLINK_PERIOD_MS);
  printf("  -h            Show this help message\n");
  printf("\nExamples:\n");
  printf("  %s                    # Blink forever with 1s period\n", 
         progname);
  printf("  %s -t 10              # Blink for 10 seconds\n", progname);
  printf("  %s -p 2000            # Blink with 2s period\n", progname);
  printf("  %s -d /dev/gpio1      # Use different GPIO\n", progname);
  printf("\nPress Ctrl+C to stop\n\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * blink_led_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd;
  int ret;
  bool led_state = false;
  int total_time = 0;        /* 0 = run forever */
  int blink_period = BLINK_PERIOD_MS;
  int elapsed_time = 0;
  int on_time;
  int off_time;
  FAR const char *devpath = GPIO_LED_DEVPATH;
  int option;

  /* Parse command line arguments */

  while ((option = getopt(argc, argv, "d:t:p:h")) != -1)
    {
      switch (option)
        {
          case 'd':
            devpath = optarg;
            break;

          case 't':
            total_time = atoi(optarg);
            if (total_time < 0)
              {
                fprintf(stderr, "ERROR: Invalid time value\n");
                return EXIT_FAILURE;
              }
            break;

          case 'p':
            blink_period = atoi(optarg);
            if (blink_period < 100)
              {
                fprintf(stderr, "ERROR: Period must be >= 100ms\n");
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

  /* Calculate ON and OFF times (50% duty cycle) */

  on_time = blink_period / 2;
  off_time = blink_period - on_time;

  /* Setup signal handler for graceful shutdown */

  signal(SIGINT, signal_handler);

  /* Print configuration */

  printf("╔════════════════════════════════════════╗\n");
  printf("║      LED Blink Application             ║\n");
  printf("╚════════════════════════════════════════╝\n");
  printf("Device      : %s\n", devpath);
  printf("Period      : %d ms\n", blink_period);
  printf("ON time     : %d ms\n", on_time);
  printf("OFF time    : %d ms\n", off_time);
  
  if (total_time > 0)
    {
      printf("Duration    : %d seconds\n", total_time);
    }
  else
    {
      printf("Duration    : Forever (press Ctrl+C to stop)\n");
    }
  
  printf("────────────────────────────────────────\n");

  /* Open the GPIO device */

  fd = open(devpath, O_RDWR);
  if (fd < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to open %s: %d\n", devpath, errcode);
      fprintf(stderr, "Make sure CONFIG_DEV_GPIO is enabled and\n");
      fprintf(stderr, "the GPIO device is initialized.\n");
      return EXIT_FAILURE;
    }

  printf("GPIO device opened successfully\n");
  printf("Starting LED blink...\n\n");

  /* Main blink loop */

  while (g_blink_running)
    {
      /* Check if we should stop based on total time */

      if (total_time > 0 && elapsed_time >= total_time * 1000)
        {
          break;
        }

      /* Toggle LED state */

      led_state = !led_state;

      /* Write to GPIO */

      ret = ioctl(fd, GPIOC_WRITE, (unsigned long)((uintptr_t)&led_state));
      if (ret < 0)
        {
          int errcode = errno;
          fprintf(stderr, "ERROR: GPIOC_WRITE failed: %d\n", errcode);
          break;
        }

      /* Print status */

      printf("\r[%6d ms] LED: %s", 
             elapsed_time, 
             led_state ? "ON ●" : "OFF ○");
      fflush(stdout);

      /* Wait */

      if (led_state)
        {
          usleep(on_time * 1000);      /* ON time */
          elapsed_time += on_time;
        }
      else
        {
          usleep(off_time * 1000);     /* OFF time */
          elapsed_time += off_time;
        }
    }

  /* Turn off LED before exiting */

  printf("\n\nTurning off LED...\n");
  led_state = false;
  ioctl(fd, GPIOC_WRITE, (unsigned long)((uintptr_t)&led_state));

  /* Close device */

  close(fd);

  printf("Total runtime: %d.%03d seconds\n", 
         elapsed_time / 1000, 
         elapsed_time % 1000);
  printf("LED blink application stopped.\n");

  return EXIT_SUCCESS;
}