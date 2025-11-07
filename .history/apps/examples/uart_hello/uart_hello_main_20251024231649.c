/****************************************************************************
 * apps/examples/uart_hello/uart_hello_main.c
 *
 * Simple UART Hello World Application for STM32F411-Minimum
 * - UART1: Send "Hello world!\n" every 1 second
 * - UART2: Debug console
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART1_DEVICE  "/dev/ttyS0"  /* USART1 */
#define HELLO_MSG     "Hello world!\n"
#define SEND_INTERVAL 1             /* 1 second */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: print_timestamp
 *
 * Description:
 *   Print timestamp to debug console
 *
 ****************************************************************************/

static void print_timestamp(int count)
{
  time_t rawtime;
  struct tm *timeinfo;
  
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  
  printf("[%02d:%02d:%02d] Message #%d sent\n",
         timeinfo->tm_hour,
         timeinfo->tm_min,
         timeinfo->tm_sec,
         count);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * uart_hello_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd_uart1;
  ssize_t written;
  int count = 0;
  int ret;

  /* Print application info to debug console (UART2) */
  printf("\n");
  printf("========================================\n");
  printf(" UART Hello World Application\n");
  printf("========================================\n");
  printf("Board: STM32F411-Minimum\n");
  printf("UART1 (ttyS0): Data output - PA9/PA10\n");
  printf("UART2 (ttyS1): Debug console - PA2/PA3\n");
  printf("Message: \"%s\"", HELLO_MSG);
  printf("Interval: %d second(s)\n", SEND_INTERVAL);
  printf("========================================\n\n");

  /* Open UART1 for writing */
  printf("Opening %s...\n", UART1_DEVICE);
  
  fd_uart1 = open(UART1_DEVICE, O_WRONLY);
  if (fd_uart1 < 0)
    {
      fprintf(stderr, "ERROR: Failed to open %s: %d\n", 
              UART1_DEVICE, errno);
      return EXIT_FAILURE;
    }

  printf("✓ UART1 opened successfully\n");
  printf("Starting transmission...\n\n");

  /* Main loop: Send message every 1 second */
  while (1)
    {
      /* Send "Hello world!\n" to UART1 */
      written = write(fd_uart1, HELLO_MSG, strlen(HELLO_MSG));
      
      if (written < 0)
        {
          fprintf(stderr, "ERROR: Write failed: %d\n", errno);
          ret = EXIT_FAILURE;
          break;
        }
      else if (written != strlen(HELLO_MSG))
        {
          fprintf(stderr, "WARNING: Partial write (%zd/%zu bytes)\n",
                  written, strlen(HELLO_MSG));
        }

      /* Increment counter */
      count++;

      /* Print status to debug console (UART2) */
      print_timestamp(count);

      /* Wait 1 second */
      sleep(SEND_INTERVAL);
    }

  /* Cleanup */
  close(fd_uart1);
  printf("UART1 closed\n");

  return ret;
}