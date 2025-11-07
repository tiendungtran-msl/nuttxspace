/****************************************************************************
 * apps/examples/timer_test/timer_test_main.c
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
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/timers/timer.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIMER_DEVNAME "/dev/timer1"
#define GPIO_DEVNAME "/dev/gpio_c13"

#define TIMER_INTERVAL 1000000  /* 1000ms in microseconds */
#define TIMER_DELAY 100000      /* Delay between samples in microseconds */
#define TIMER_NSAMPLES 20       /* Number of samples before stopping */
#define TIMER_SIGNO 32          /* Signal number for timer notifications */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_fd_gpio;
static volatile unsigned long g_nsignals;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void timer_sighandler(int signo, FAR siginfo_t *siginfo,
                             FAR void *context)
{
  unsigned long value;
  int ret;

  /* Read current GPIO state */
  ret = ioctl(g_fd_gpio, GPIOC_READ, (unsigned long)&value);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to read GPIO state: %d\n", errno);
      return;
    }

  /* Toggle the GPIO state */
  value = !value;
  ret = ioctl(g_fd_gpio, GPIOC_WRITE, value);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to write GPIO state: %d\n", errno);
      return;
    }

  /* Increment signal count for tracking */
  g_nsignals++;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * timer_test_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct timer_notify_s notify;
  struct sigaction act;
  int ret;
  int fd;
  int i;
  unsigned long direction = GPIO_OUTPUT;
  unsigned long initial_value = 0;

  /* Open the GPIO device */
  printf("Open %s\n", GPIO_DEVNAME);
  g_fd_gpio = open(GPIO_DEVNAME, O_RDWR);
  if (g_fd_gpio < 0)
    {
      fprintf(stderr, "ERROR: Failed to open %s: %d\n", GPIO_DEVNAME, errno);
      return EXIT_FAILURE;
    }

  /* Set GPIO direction to output */
  ret = ioctl(g_fd_gpio, GPIOC_SETDIRECTION, direction);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to set GPIO direction: %d\n", errno);
      close(g_fd_gpio);
      return EXIT_FAILURE;
    }

  /* Set initial GPIO state to low */
  ret = ioctl(g_fd_gpio, GPIOC_WRITE, initial_value);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to set initial GPIO state: %d\n", errno);
      close(g_fd_gpio);
      return EXIT_FAILURE;
    }

  /* Open the timer device */
  printf("Open %s\n", TIMER_DEVNAME);
  fd = open(TIMER_DEVNAME, O_RDONLY);
  if (fd < 0)
    {
      fprintf(stderr, "ERROR: Failed to open %s: %d\n", TIMER_DEVNAME, errno);
      close(g_fd_gpio);
      return EXIT_FAILURE;
    }

  /* Set the timer interval */
  printf("Set timer interval to %lu\n", (unsigned long)TIMER_INTERVAL);
  ret = ioctl(fd, TCIOC_SETTIMEOUT, TIMER_INTERVAL);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to set the timer interval: %d\n", errno);
      close(fd);
      close(g_fd_gpio);
      return EXIT_FAILURE;
    }

  /* Attach the signal handler */
  g_nsignals = 0;
  act.sa_sigaction = timer_sighandler;
  act.sa_flags = SA_SIGINFO;
  sigfillset(&act.sa_mask);
  sigdelset(&act.sa_mask, TIMER_SIGNO);
  ret = sigaction(TIMER_SIGNO, &act, NULL);
  if (ret != OK)
    {
      fprintf(stderr, "ERROR: sigaction failed: %d\n", errno);
      close(fd);
      close(g_fd_gpio);
      return EXIT_FAILURE;
    }

  /* Register the notification callback */
  printf("Attach timer handler\n");
  notify.pid = getpid();
  notify.periodic = true;
  notify.event.sigev_notify = SIGEV_SIGNAL;
  notify.event.sigev_signo = TIMER_SIGNO;
  notify.event.sigev_value.sival_ptr = NULL;
  ret = ioctl(fd, TCIOC_NOTIFICATION, (unsigned long)((uintptr_t)&notify));
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to set the timer handler: %d\n", errno);
      close(fd);
      close(g_fd_gpio);
      return EXIT_FAILURE;
    }

  /* Start the timer */
  printf("Start the timer\n");
  ret = ioctl(fd, TCIOC_START, 0);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to start the timer: %d\n", errno);
      close(fd);
      close(g_fd_gpio);
      return EXIT_FAILURE;
    }

  /* Wait and sample for a bit */
  for (i = 0; i < TIMER_NSAMPLES; i++)
    {
      usleep(TIMER_DELAY);
      printf("Signals received: %lu\n", g_nsignals);
    }

  /* Stop the timer */
  printf("Stop the timer\n");
  ret = ioctl(fd, TCIOC_STOP, 0);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to stop the timer: %d\n", errno);
    }

  /* Detach the signal handler */
  act.sa_handler = SIG_DFL;
  sigaction(TIMER_SIGNO, &act, NULL);

  /* Close the devices */
  printf("Finished\n");
  close(fd);
  close(g_fd_gpio);
  return EXIT_SUCCESS;
}