/****************************************************************************
 * apps/examples/pwm_example/pwm_example_main.c
 *
 * Simple PWM Example for NuttX
 * Creates PWM signal on /dev/pwm2
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <nuttx/timers/pwm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PWM_DEVICE_PATH "/dev/pwm2"
#define DEFAULT_FREQUENCY 1000    /* 1 KHz */
#define DEFAULT_DUTY      50      /* 50% duty cycle */
#define DEFAULT_DURATION  5       /* 5 seconds */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(const char *progname)
{
  printf("Usage: %s [OPTIONS]\n", progname);
  printf("Options:\n");
  printf("  -f <freq>     Set frequency in Hz (default: %d)\n", DEFAULT_FREQUENCY);
  printf("  -d <duty>     Set duty cycle percentage 0-100 (default: %d)\n", DEFAULT_DUTY);
  printf("  -t <time>     Duration in seconds (default: %d)\n", DEFAULT_DURATION);
  printf("  -h            Show this help\n");
  printf("\nExample:\n");
  printf("  %s -f 2000 -d 75 -t 10\n", progname);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct pwm_info_s pwm_info;
  int fd;
  int ret;
  int opt;
  uint32_t frequency = DEFAULT_FREQUENCY;
  uint8_t duty = DEFAULT_DUTY;
  uint32_t duration = DEFAULT_DURATION;

  printf("PWM Example Application\n");
  printf("Device: %s\n", PWM_DEVICE_PATH);

  /* Parse command line arguments */
  while ((opt = getopt(argc, argv, "f:d:t:h")) != -1)
    {
      switch (opt)
        {
          case 'f':
            frequency = (uint32_t)atoi(optarg);
            if (frequency == 0)
              {
                fprintf(stderr, "Invalid frequency: %s\n", optarg);
                return EXIT_FAILURE;
              }
            break;

          case 'd':
            duty = (uint8_t)atoi(optarg);
            if (duty > 100)
              {
                fprintf(stderr, "Duty cycle must be 0-100%%\n");
                return EXIT_FAILURE;
              }
            break;

          case 't':
            duration = (uint32_t)atoi(optarg);
            if (duration == 0)
              {
                fprintf(stderr, "Invalid duration: %s\n", optarg);
                return EXIT_FAILURE;
              }
            break;

          case 'h':
            show_usage(argv[0]);
            return EXIT_SUCCESS;

          default:
            show_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

  printf("Configuration:\n");
  printf("  Frequency: %lu Hz\n", (unsigned long)frequency);
  printf("  Duty Cycle: %d%%\n", duty);
  printf("  Duration: %lu seconds\n", (unsigned long)duration);

  /* Open PWM device */
  fd = open(PWM_DEVICE_PATH, O_RDONLY);
  if (fd < 0)
    {
      fprintf(stderr, "ERROR: Failed to open %s: %d\n", PWM_DEVICE_PATH, errno);
      return EXIT_FAILURE;
    }

  printf("PWM device opened successfully\n");

  /* Configure PWM parameters */
  pwm_info.frequency = frequency;
  pwm_info.duty = (duty * 65536) / 100; /* Convert percentage to 16-bit value */

  printf("Setting PWM parameters...\n");
  ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)&pwm_info);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to set PWM characteristics: %d\n", errno);
      close(fd);
      return EXIT_FAILURE;
    }

  /* Start PWM */
  printf("Starting PWM...\n");
  ret = ioctl(fd, PWMIOC_START, 0);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to start PWM: %d\n", errno);
      close(fd);
      return EXIT_FAILURE;
    }

  printf("PWM started successfully!\n");
  printf("Generating PWM signal for %lu seconds...\n", (unsigned long)duration);

  /* Run for specified duration */
  sleep(duration);

  /* Stop PWM */
  printf("Stopping PWM...\n");
  ret = ioctl(fd, PWMIOC_STOP, 0);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to stop PWM: %d\n", errno);
    }
  else
    {
      printf("PWM stopped successfully\n");
    }

  /* Close device */
  close(fd);
  printf("PWM Example completed\n");

  return EXIT_SUCCESS;
}