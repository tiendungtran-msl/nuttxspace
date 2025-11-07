/****************************************************************************
 * apps/examples/gpio_v2/gpio_v2_main.c
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
#include <errno.h>
#include <unistd.h>
#include <nuttx/ioexpander/gpio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_DEVICE_PATH "/dev/gpio_c13"
#define BLINK_INTERVAL_US (500 * 1000) /* 500ms */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(FAR const char *progname)
{
  fprintf(stderr, "USAGE: %s <command>\n", progname);
  fprintf(stderr, "Commands:\n");
  fprintf(stderr, "\thigh: Set PC13 to high (1)\n");
  fprintf(stderr, "\tlow:  Set PC13 to low (0)\n");
  fprintf(stderr, "\tblink: Blink PC13 (toggle every 500ms)\n");
  fprintf(stderr, "\t-h:   Show this help message\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

 void handler_cmd(FAR char *)

/****************************************************************************
 * gpio_v2_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd;
  int ret;
  bool invalue;
  enum gpio_pintype_e pintype;

  /* Kiểm tra đối số */

  if (argc != 2)
    {
      fprintf(stderr, "ERROR: Exactly one command is required\n");
      show_usage(argv[0]);
      return EXIT_FAILURE;
    }

  if (strcmp(argv[1], "-h") == 0)
    {
      show_usage(argv[0]);
      return EXIT_SUCCESS;
    }

  if (strcmp(argv[1], "high") != 0 &&
      strcmp(argv[1], "low") != 0 &&
      strcmp(argv[1], "blink") != 0)
    {
      fprintf(stderr, "ERROR: Invalid command: %s\n", argv[1]);
      show_usage(argv[0]);
      return EXIT_FAILURE;
    }

  /* Mở driver GPIO */

  fd = open(GPIO_DEVICE_PATH, O_RDWR);
  if (fd < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to open %s: %d\n", GPIO_DEVICE_PATH, errcode);
      return EXIT_FAILURE;
    }

  /* Đặt chân chế độ OUTPUT */

  ret = ioctl(fd, GPIOC_SETPINTYPE, (unsigned long)GPIO_OUTPUT_PIN);
  if (ret < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to set pintype on %s: %d\n", GPIO_DEVICE_PATH, errcode);
      close(fd);
      return EXIT_FAILURE;
    }

  /* Kiểm tra kiểu pin */

  ret = ioctl(fd, GPIOC_PINTYPE, (unsigned long)((uintptr_t)&pintype));
  if (ret < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to read pintype from %s: %d\n", GPIO_DEVICE_PATH, errcode);
      close(fd);
      return EXIT_FAILURE;
    }

  /* Kiểm tra loại pin */
  if (pintype != GPIO_OUTPUT_PIN)
    {
      fprintf(stderr, "ERROR: Pin %s is not configured as output: pintype=%d\n",
              GPIO_DEVICE_PATH, pintype);
      close(fd);
      return EXIT_FAILURE;
    }

  /* Đọc giá trị mặc định pin */

  ret = ioctl(fd, GPIOC_READ, (unsigned long)((uintptr_t)&invalue));
  if (ret < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to read value from %s: %d\n", GPIO_DEVICE_PATH, errcode);
      close(fd);
      return EXIT_FAILURE;
    }

  printf("Initial state of %s: %u\n", GPIO_DEVICE_PATH, (unsigned int)invalue);

  /* ======================================================================================= */
  /* Handle commands */

  if (strcmp(argv[1], "high") == 0)
    {
      ret = ioctl(fd, GPIOC_WRITE, (unsigned long)true);
      if (ret < 0)
        {
          int errcode = errno;
          fprintf(stderr, "ERROR: Failed to set %s to high: %d\n", GPIO_DEVICE_PATH, errcode);
          close(fd);
          return EXIT_FAILURE;
        }

      ret = ioctl(fd, GPIOC_READ, (unsigned long)((uintptr_t)&invalue));
      if (ret < 0)
        {
          int errcode = errno;
          fprintf(stderr, "ERROR: Failed to verify value from %s: %d\n", GPIO_DEVICE_PATH, errcode);
          close(fd);
          return EXIT_FAILURE;
        }

      printf("Set %s to high. Verified value: %u\n", GPIO_DEVICE_PATH, (unsigned int)invalue);
    }
  else if (strcmp(argv[1], "low") == 0)
    {
      ret = ioctl(fd, GPIOC_WRITE, (unsigned long)false);
      if (ret < 0)
        {
          int errcode = errno;
          fprintf(stderr, "ERROR: Failed to set %s to low: %d\n", GPIO_DEVICE_PATH, errcode);
          close(fd);
          return EXIT_FAILURE;
        }

      ret = ioctl(fd, GPIOC_READ, (unsigned long)((uintptr_t)&invalue));
      if (ret < 0)
        {
          int errcode = errno;
          fprintf(stderr, "ERROR: Failed to verify value from %s: %d\n", GPIO_DEVICE_PATH, errcode);
          close(fd);
          return EXIT_FAILURE;
        }

      printf("Set %s to low. Verified value: %u\n", GPIO_DEVICE_PATH, (unsigned int)invalue);
    }
  else if (strcmp(argv[1], "blink") == 0)
    {
      printf("Blinking %s (press Ctrl+C to stop)\n", GPIO_DEVICE_PATH);

      while (true)
        {
          /* Toggle pin state */
          ret = ioctl(fd, GPIOC_WRITE, (unsigned long)!invalue);
          if (ret < 0)
            {
              int errcode = errno;
              fprintf(stderr, "ERROR: Failed to toggle %s: %d\n", GPIO_DEVICE_PATH, errcode);
              close(fd);
              return EXIT_FAILURE;
            }

          /* Read back to verify */
          ret = ioctl(fd, GPIOC_READ, (unsigned long)((uintptr_t)&invalue));
          if (ret < 0)
            {
              int errcode = errno;
              fprintf(stderr, "ERROR: Failed to verify value from %s: %d\n", GPIO_DEVICE_PATH, errcode);
              close(fd);
              return EXIT_FAILURE;
            }

          printf("Toggled %s to: %u\n", GPIO_DEVICE_PATH, (unsigned int)invalue);

          /* Wait for 500ms */
          usleep(BLINK_INTERVAL_US);
        }
    }

  /* Close the driver */
  close(fd);
  return EXIT_SUCCESS;
}