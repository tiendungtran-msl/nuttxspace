/****************************************************************************
 * apps/examples/gpio_int/gpio_int_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <nuttx/ioexpander/gpio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_INPUT_PATH "/dev/gpio_a2"   /* PA2 for interrupt input */
#define GPIO_OUTPUT_PATH "/dev/gpio_c13" /* PC13 for output toggle */
#define INTERRUPT_SIGNAL SIGUSR1         /* Signal number for interrupt */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(FAR const char *progname)
{
  fprintf(stderr, "USAGE: %s\n", progname);
  fprintf(stderr, "This program waits for interrupts on PA2 and toggles PC13 on each interrupt.\n");
  fprintf(stderr, "\t-h: Show this help message\n");
}

/* Signal handler for interrupt */
static void interrupt_handler(int signo)
{
  if (signo == INTERRUPT_SIGNAL)
    {
      printf("Interrupt received on PA2!\n");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * gpio_int_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd_input;  /* File descriptor for PA2 */
  int fd_output; /* File descriptor for PC13 */
  int ret;
  bool current_value;
  enum gpio_pintype_e pintype;
  struct sigaction act;
  sigset_t set;

  /* Parse command line arguments */

  if (argc > 1 && strcmp(argv[1], "-h") == 0)
    {
      show_usage(argv[0]);
      return EXIT_SUCCESS;
    }

  /* Open the input GPIO driver (PA2) */

  fd_input = open(GPIO_INPUT_PATH, O_RDWR);
  if (fd_input < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to open %s: %d\n", GPIO_INPUT_PATH, errcode);
      return EXIT_FAILURE;
    }

  /* Open the output GPIO driver (PC13) */

  fd_output = open(GPIO_OUTPUT_PATH, O_RDWR);
  if (fd_output < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to open %s: %d\n", GPIO_OUTPUT_PATH, errcode);
      close(fd_input);
      return EXIT_FAILURE;
    }

  /* Set PA2 as interrupt on falling edge (since pull-up, button press pulls low) */

  ret = ioctl(fd_input, GPIOC_SETPINTYPE, (unsigned long)GPIO_INTERRUPT_FALLING_PIN);
  if (ret < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to set pintype on %s: %d\n", GPIO_INPUT_PATH, errcode);
      close(fd_input);
      close(fd_output);
      return EXIT_FAILURE;
    }

  /* Verify PA2 pin type */

  ret = ioctl(fd_input, GPIOC_PINTYPE, (unsigned long)((uintptr_t)&pintype));
  if (ret < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to read pintype from %s: %d\n", GPIO_INPUT_PATH, errcode);
      close(fd_input);
      close(fd_output);
      return EXIT_FAILURE;
    }

  if (pintype != GPIO_INTERRUPT_FALLING_PIN)
    {
      fprintf(stderr, "ERROR: Pin %s is not configured as falling edge interrupt: pintype=%d\n",
              GPIO_INPUT_PATH, pintype);
      close(fd_input);
      close(fd_output);
      return EXIT_FAILURE;
    }

  /* Set PC13 as output */

  ret = ioctl(fd_output, GPIOC_SETPINTYPE, (unsigned long)GPIO_OUTPUT_PIN);
  if (ret < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to set pintype on %s: %d\n", GPIO_OUTPUT_PATH, errcode);
      close(fd_input);
      close(fd_output);
      return EXIT_FAILURE;
    }

  /* Verify PC13 pin type */

  ret = ioctl(fd_output, GPIOC_PINTYPE, (unsigned long)((uintptr_t)&pintype));
  if (ret < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to read pintype from %s: %d\n", GPIO_OUTPUT_PATH, errcode);
      close(fd_input);
      close(fd_output);
      return EXIT_FAILURE;
    }

  if (pintype != GPIO_OUTPUT_PIN)
    {
      fprintf(stderr, "ERROR: Pin %s is not configured as output: pintype=%d\n",
              GPIO_OUTPUT_PATH, pintype);
      close(fd_input);
      close(fd_output);
      return EXIT_FAILURE;
    }

  /* Read initial state of PC13 */

  ret = ioctl(fd_output, GPIOC_READ, (unsigned long)((uintptr_t)&current_value));
  if (ret < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to read initial value from %s: %d\n", GPIO_OUTPUT_PATH, errcode);
      close(fd_input);
      close(fd_output);
      return EXIT_FAILURE;
    }

  printf("Initial state of %s: %u\n", GPIO_OUTPUT_PATH, (unsigned int)current_value);

  /* Set up signal handler for interrupt */

  act.sa_handler = interrupt_handler;
  act.sa_flags = 0;
  sigemptyset(&act.sa_mask);

  ret = sigaction(INTERRUPT_SIGNAL, &act, NULL);
  if (ret < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to set signal handler: %d\n", errcode);
      close(fd_input);
      close(fd_output);
      return EXIT_FAILURE;
    }

  /* Register the signal for the interrupt pin */

  struct sigevent notify;
  notify.sigev_notify = SIGEV_SIGNAL;
  notify.sigev_signo = INTERRUPT_SIGNAL;

  ret = ioctl(fd_input, GPIOC_REGISTER, (unsigned long)&notify);
  if (ret < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to register interrupt on %s: %d\n", GPIO_INPUT_PATH, errcode);
      close(fd_input);
      close(fd_output);
      return EXIT_FAILURE;
    }

  printf("Waiting for interrupts on %s... (Press Ctrl+C to stop)\n", GPIO_INPUT_PATH);

  /* Main loop: Wait for signals and toggle PC13 */

  sigemptyset(&set);
  sigaddset(&set, INTERRUPT_SIGNAL);

  while (true)
    {
      int signo;

      ret = sigwait(&set, &signo);
      if (ret == 0 && signo == INTERRUPT_SIGNAL)
        {
          /* Toggle PC13 */

          current_value = !current_value;

          ret = ioctl(fd_output, GPIOC_WRITE, (unsigned long)current_value);
          if (ret < 0)
            {
              int errcode = errno;
              fprintf(stderr, "ERROR: Failed to toggle %s: %d\n", GPIO_OUTPUT_PATH, errcode);
              break;
            }

          /* Read back to verify */

          ret = ioctl(fd_output, GPIOC_READ, (unsigned long)((uintptr_t)&current_value));
          if (ret < 0)
            {
              int errcode = errno;
              fprintf(stderr, "ERROR: Failed to verify value from %s: %d\n", GPIO_OUTPUT_PATH, errcode);
              break;
            }

          printf("Toggled %s to: %u\n", GPIO_OUTPUT_PATH, (unsigned int)current_value);
        }
      else
        {
          fprintf(stderr, "ERROR: Failed to wait for signal: %d\n", ret);
          break;
        }
    }

  /* Unregister interrupt and close drivers */

  ioctl(fd_input, GPIOC_UNREGISTER, 0);
  close(fd_input);
  close(fd_output);

  return EXIT_SUCCESS;
}