/****************************************************************************
 * apps/examples/gpio_pc13/gpio_pc13_main.c
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
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <nuttx/ioexpander/gpio.h>
#include <unistd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_PC13_DEVICE "/dev/gpio1"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(FAR const char *progname)
{
  printf("Usage: %s <command>\n", progname);
  printf("Commands:\n");
  printf("  on     - Turn on PC13 (set high)\n");
  printf("  off    - Turn off PC13 (set low)\n");
  printf("  toggle - Toggle PC13 state\n");
  printf("  read   - Read PC13 state\n");
  printf("  blink  - Blink PC13 10 times\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd;
  int ret;
  bool value;
  enum gpio_pintype_e pintype;

  if (argc < 2)
    {
      show_usage(argv[0]);
      return EXIT_FAILURE;
    }

  /* Open GPIO device */
  fd = open(GPIO_PC13_DEVICE, O_RDWR);
  if (fd < 0)
    {
      printf("ERROR: Failed to open %s: %d\n", GPIO_PC13_DEVICE, fd);
      printf("Make sure GPIO driver is enabled and PC13 is configured\n");
      printf("Check board configuration and ensure CONFIG_DEV_GPIO=y\n");
      return EXIT_FAILURE;
    }

  /* Get pin type to verify configuration */
  ret = ioctl(fd, GPIOC_PINTYPE, (unsigned long)&pintype);
  if (ret < 0)
    {
      printf("ERROR: Failed to get pin type: %d\n", ret);
      close(fd);
      return EXIT_FAILURE;
    }

  printf("PC13 pin type: %s\n", 
         pintype == GPIO_OUTPUT_PIN ? "OUTPUT" :
         pintype == GPIO_INPUT_PIN ? "INPUT" :
         pintype == GPIO_INTERRUPT_PIN ? "INTERRUPT" : "UNKNOWN");

  /* Process command */
  if (strcmp(argv[1], "on") == 0)
    {
      value = true;
      ret = ioctl(fd, GPIOC_WRITE, (unsigned long)value);
      if (ret < 0)
        {
          printf("ERROR: Failed to write GPIO: %d\n", ret);
        }
      else
        {
          printf("PC13 turned ON\n");
        }
    }
  else if (strcmp(argv[1], "off") == 0)
    {
      value = false;
      ret = ioctl(fd, GPIOC_WRITE, (unsigned long)value);
      if (ret < 0)
        {
          printf("ERROR: Failed to write GPIO: %d\n", ret);
        }
      else
        {
          printf("PC13 turned OFF\n");
        }
    }
  else if (strcmp(argv[1], "toggle") == 0)
    {
      /* Read current state */
      ret = ioctl(fd, GPIOC_READ, (unsigned long)&value);
      if (ret < 0)
        {
          printf("ERROR: Failed to read GPIO: %d\n", ret);
        }
      else
        {
          /* Toggle state */
          value = !value;
          ret = ioctl(fd, GPIOC_WRITE, (unsigned long)value);
          if (ret < 0)
            {
              printf("ERROR: Failed to write GPIO: %d\n", ret);
            }
          else
            {
              printf("PC13 toggled to %s\n", value ? "ON" : "OFF");
            }
        }
    }
  else if (strcmp(argv[1], "read") == 0)
    {
      ret = ioctl(fd, GPIOC_READ, (unsigned long)&value);
      if (ret < 0)
        {
          printf("ERROR: Failed to read GPIO: %d\n", ret);
        }
      else
        {
          printf("PC13 state: %s\n", value ? "HIGH" : "LOW");
        }
    }
  else if (strcmp(argv[1], "blink") == 0)
    {
      printf("Blinking PC13 10 times...\n");
      for (int i = 0; i < 10; i++)
        {
          /* Turn on */
          value = true;
          ret = ioctl(fd, GPIOC_WRITE, (unsigned long)value);
          if (ret < 0)
            {
              printf("ERROR: Failed to write GPIO: %d\n", ret);
              break;
            }
          usleep(500000); /* 500ms */

          /* Turn off */
          value = false;
          ret = ioctl(fd, GPIOC_WRITE, (unsigned long)value);
          if (ret < 0)
            {
              printf("ERROR: Failed to write GPIO: %d\n", ret);
              break;
            }
          usleep(500000); /* 500ms */
          
          printf("Blink %d/10\n", i + 1);
        }
      printf("Blink complete\n");
    }
  else
    {
      printf("ERROR: Unknown command '%s'\n", argv[1]);
      show_usage(argv[0]);
      close(fd);
      return EXIT_FAILURE;
    }

  close(fd);
  return EXIT_SUCCESS;
}