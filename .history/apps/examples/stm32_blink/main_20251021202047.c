/****************************************************************************
 * apps/examples/stm32_blink/main.c
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <nuttx/ioexpander/gpio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_PATH "/dev/gpio_c13"
#define LED_INTERVAL_MS 1000
#define PRINT_INTERVAL_MS 2000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int gpio_fd = -1;
static bool running = true;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void *led_thread(void *arg)
{
  bool led_state = false;
  
  printf("LED thread started\n");
  
  while (running)
    {
      /* Toggle LED */
      if (ioctl(gpio_fd, GPIOC_WRITE, (unsigned long)led_state) < 0)
        {
          fprintf(stderr, "ERROR: Failed to write GPIO\n");
        }
      
      led_state = !led_state;
      usleep(LED_INTERVAL_MS * 1000);
    }
  
  return NULL;
}

static void *print_thread(void *arg)
{
  printf("Print thread started\n");
  
  while (running)
    {
      printf("Hello world!\n");
      fflush(stdout);
      usleep(PRINT_INTERVAL_MS * 1000);
    }
  
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  pthread_t led_tid;
  pthread_t print_tid;
  int ret;
  
  printf("STM32F411 Minimum - GPIO Blink & Hello World Demo\n");
  printf("=================================================\n");
  
  /* Open GPIO device */
  gpio_fd = open(GPIO_PATH, O_RDWR);
  if (gpio_fd < 0)
    {
      fprintf(stderr, "ERROR: Failed to open %s\n", GPIO_PATH);
      fprintf(stderr, "Make sure GPIO driver is enabled in kernel config\n");
      return EXIT_FAILURE;
    }
  
  printf("GPIO device %s opened successfully\n", GPIO_PATH);
  
  /* Set GPIO as output */
  ret = ioctl(gpio_fd, GPIOC_SETPINTYPE, (unsigned long)GPIO_OUTPUT_PIN);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to set GPIO as output\n");
      close(gpio_fd);
      return EXIT_FAILURE;
    }
  
  /* Create LED blink thread */
  ret = pthread_create(&led_tid, NULL, led_thread, NULL);
  if (ret != 0)
    {
      fprintf(stderr, "ERROR: Failed to create LED thread\n");
      close(gpio_fd);
      return EXIT_FAILURE;
    }
  
  /* Create print thread */
  ret = pthread_create(&print_tid, NULL, print_thread, NULL);
  if (ret != 0)
    {
      fprintf(stderr, "ERROR: Failed to create print thread\n");
      running = false;
      pthread_join(led_tid, NULL);
      close(gpio_fd);
      return EXIT_FAILURE;
    }
  
  printf("All threads created successfully\n");
  printf("Press Ctrl+C to exit (if using console)\n\n");
  
  /* Wait for threads */
  pthread_join(led_tid, NULL);
  pthread_join(print_tid, NULL);
  
  /* Cleanup */
  close(gpio_fd);
  printf("\nApplication terminated\n");
  
  return EXIT_SUCCESS;
}