/****************************************************************************
 * apps/examples/adc_reader/adc_reader_main.c
 *
 * Simple ADC reader application for STM32F411-minimum
 * Reads potentiometer value from ADC1 Channel 1 (PA1)
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <nuttx/analog/adc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC_DEVPATH   "/dev/adc0"
#define ADC_GROUPSIZE 1

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct adc_msg_s g_samples[ADC_GROUPSIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_devpath
 ****************************************************************************/

static void adc_devpath(FAR char *devpath, int devno)
{
  snprintf(devpath, 12, "/dev/adc%d", devno);
}

/****************************************************************************
 * Name: adc_read_samples
 ****************************************************************************/

static int adc_read_samples(int fd)
{
  ssize_t nbytes;
  int i;
  int nsamples;

  /* Read ADC samples */
  
  nbytes = read(fd, g_samples, ADC_GROUPSIZE * sizeof(struct adc_msg_s));
  if (nbytes < 0)
    {
      printf("ERROR: read() failed: %d\n", (int)nbytes);
      return -1;
    }
  else if (nbytes == 0)
    {
      printf("No data read\n");
      return 0;
    }

  /* Calculate number of samples read */
  
  nsamples = nbytes / sizeof(struct adc_msg_s);
  if (nsamples * sizeof(struct adc_msg_s) != nbytes)
    {
      printf("WARNING: read size (%ld) is not a multiple of sample size (%d), "
             "discarding %d bytes\n",
             (long)nbytes, sizeof(struct adc_msg_s),
             (int)(nbytes % sizeof(struct adc_msg_s)));
    }

  /* Print the samples */
  
  for (i = 0; i < nsamples; i++)
    {
      printf("Channel %d: Raw Value = %d, ", 
             g_samples[i].am_channel, g_samples[i].am_data);
      
      /* Convert to voltage (assuming 3.3V reference and 12-bit ADC) */
      float voltage = (g_samples[i].am_data * 3.3f) / 4095.0f;
      printf("Voltage = %.3fV\n", voltage);
    }

  return nsamples;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_reader_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  char devpath[12];
  int fd;
  int ret;
  int samples_read;
  int total_samples = 0;
  int max_samples = 0;

  printf("ADC Reader Application\n");
  printf("Reading from ADC1 Channel 1 (PA1)\n");
  printf("Press Ctrl+C to stop\n\n");

  /* Parse command line arguments */
  
  if (argc > 1)
    {
      max_samples = atoi(argv[1]);
      printf("Will read %d samples\n", max_samples);
    }
  else
    {
      printf("Reading continuously...\n");
    }

  /* Get the ADC device path */
  
  adc_devpath(devpath, 0);  /* Use ADC0 device */

  /* Open the ADC device for reading */
  
  fd = open(devpath, O_RDONLY);
  if (fd < 0)
    {
      printf("ERROR: open %s failed: %d\n", devpath, fd);
      return -1;
    }

  printf("Opened ADC device: %s\n\n", devpath);

  /* Read samples in a loop */
  
  while (1)
    {
      samples_read = adc_read_samples(fd);
      if (samples_read < 0)
        {
          printf("ERROR: Failed to read ADC samples\n");
          break;
        }
      
      total_samples += samples_read;
      
      /* Check if we've read enough samples */
      
      if (max_samples > 0 && total_samples >= max_samples)
        {
          printf("\nRead %d samples, exiting...\n", total_samples);
          break;
        }

      /* Wait a bit before next reading */
      
      usleep(500000);  /* 500ms delay */
    }

  /* Close the ADC device */
  
  close(fd);
  printf("ADC Reader finished\n");
  return 0;
}

/****************************************************************************
 * Kconfig and Makefile configuration
 ****************************************************************************/

/*
 * Add this to your apps/examples/Kconfig:
 *
 * config EXAMPLES_ADC_READER
 *   tristate "ADC Reader example"
 *   default n
 *   depends on ADC
 *   ---help---
 *     Enable the ADC reader example
 *
 * if EXAMPLES_ADC_READER
 *
 * config EXAMPLES_ADC_READER_PROGNAME
 *   string "Program name"
 *   default "adc_reader"
 *   ---help---
 *     This is the name of the program that will be used when the NSH ELF
 *     program is installed.
 *
 * config EXAMPLES_ADC_READER_PRIORITY
 *   int "ADC reader task priority"
 *   default 100
 *
 * config EXAMPLES_ADC_READER_STACKSIZE
 *   int "ADC reader stack size"
 *   default 2048
 *
 * endif
 */

/*
 * Create Makefile (apps/examples/adc_reader/Makefile):
 *
 * include $(APPDIR)/Make.defs
 *
 * # ADC Reader built-in application info
 *
 * PROGNAME = $(CONFIG_EXAMPLES_ADC_READER_PROGNAME)
 * PRIORITY = $(CONFIG_EXAMPLES_ADC_READER_PRIORITY)
 * STACKSIZE = $(CONFIG_EXAMPLES_ADC_READER_STACKSIZE)
 * MODULE = $(CONFIG_EXAMPLES_ADC_READER)
 *
 * # ADC Reader Example
 *
 * MAINSRC = adc_reader_main.c
 *
 * include $(APPDIR)/Application.mk
 */