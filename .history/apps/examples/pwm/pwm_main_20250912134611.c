/****************************************************************************
 * pwm_main.c
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
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <inttypes.h>
#include <nuttx/timers/pwm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEFAULT_DEVPATH   CONFIG_EXAMPLES_PWM_DEVPATH   /* Đường dẫn mặc định, ví dụ: /dev/pwm2 */
#define DEFAULT_FREQUENCY CONFIG_EXAMPLES_PWM_FREQUENCY /* Tần số mặc định (Hz) */
#define DEFAULT_DUTY      CONFIG_EXAMPLES_PWM_DUTYPCT   /* Chu kỳ nhiệm vụ mặc định (%) */
#define DEFAULT_DURATION  CONFIG_EXAMPLES_PWM_DURATION   /* Thời gian chạy mặc định (giây) */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pwm_state_s {
  bool      initialized; /* Cờ khởi tạo */
  char     *devpath;     /* Đường dẫn thiết bị PWM */
  uint32_t  freq;        /* Tần số PWM (Hz) */
  uint8_t   duty;        /* Chu kỳ nhiệm vụ (%) */
  int       duration;    /* Thời gian chạy (giây) */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pwm_state_s g_pwmstate;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Cập nhật đường dẫn thiết bị PWM */
static void pwm_devpath(struct pwm_state_s *pwm, const char *devpath) {
  if (pwm->devpath) {
    free(pwm->devpath);
  }
  pwm->devpath = strdup(devpath);
}

/* Hiển thị hướng dẫn sử dụng */
static void pwm_help(struct pwm_state_s *pwm) {
  printf("Usage: pwm [OPTIONS]\n");
  printf("\nOptions:\n");
  printf("  -p <devpath>  : PWM device path (default: %s, current: %s)\n",
         DEFAULT_DEVPATH, pwm->devpath ? pwm->devpath : "NONE");
  printf("  -f <freq>     : PWM frequency in Hz (default: %d, current: %" PRIu32 ")\n",
         DEFAULT_FREQUENCY, pwm->freq);
  printf("  -d <duty>     : Duty cycle in %% (0-100, default: %d, current: %d)\n",
         DEFAULT_DUTY, pwm->duty);
  printf("  -t <duration> : Duration in seconds (default: %d, current: %d)\n",
         DEFAULT_DURATION, pwm->duration);
  printf("  -h            : Show this help message\n");
}

/* Phân tích tham số dạng chuỗi */
static int arg_string(char **arg, char **value) {
  char *ptr = *arg;
  if (ptr[2] == '\0') {
    *value = arg[1];
    return 2;
  } else {
    *value = &ptr[2];
    return 1;
  }
}

/* Phân tích tham số dạng số */
static int arg_decimal(char **arg, long *value) {
  char *string;
  int ret = arg_string(arg, &string);
  *value = strtol(string, NULL, 10);
  return ret;
}

/* Phân tích tham số dòng lệnh */
static void parse_args(struct pwm_state_s *pwm, int argc, char **argv) {
  long value;
  int index, nargs;

  for (index = 1; index < argc;) {
    char *ptr = argv[index];
    if (ptr[0] != '-') {
      printf("Invalid option: %s\n", ptr);
      pwm_help(pwm);
      exit(1);
    }

    switch (ptr[1]) {
      case 'f': /* Tần số */
        nargs = arg_decimal(&argv[index], &value);
        if (value < 1) {
          printf("Frequency out of range: %ld\n", value);
          exit(1);
        }
        pwm->freq = (uint32_t)value;
        index += nargs;
        break;

      case 'd': /* Chu kỳ nhiệm vụ */
        nargs = arg_decimal(&argv[index], &value);
        if (value < 0 || value > 100) {
          printf("Duty out of range: %ld\n", value);
          exit(1);
        }
        pwm->duty = (uint8_t)value;
        index += nargs;
        break;

      case 'p': /* Đường dẫn thiết bị */
        nargs = arg_string(&argv[index], &ptr);
        pwm_devpath(pwm, ptr);
        index += nargs;
        break;

      case 't': /* Thời gian chạy */
        nargs = arg_decimal(&argv[index], &value);
        if (value < 1 || value > INT_MAX) {
          printf("Duration out of range: %ld\n", value);
          exit(1);
        }
        pwm->duration = (int)value;
        index += nargs;
        break;

      case 'h': /* Hiển thị trợ giúp */
        pwm_help(pwm);
        exit(0);

      default:
        printf("Unsupported option: %s\n", ptr);
        pwm_help(pwm);
        exit(1);
    }
  }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char *argv[]) {
  struct pwm_info_s info;
  int fd, ret;

  /* Khởi tạo trạng thái */
  if (!g_pwmstate.initialized) {
    g_pwmstate.duty = DEFAULT_DUTY;
    g_pwmstate.freq = DEFAULT_FREQUENCY;
    g_pwmstate.duration = DEFAULT_DURATION;
    g_pwmstate.initialized = true;
  }

  /* Phân tích tham số dòng lệnh */
  parse_args(&g_pwmstate, argc, argv);

  /* Sử dụng đường dẫn mặc định nếu chưa được chỉ định */
  if (!g_pwmstate.devpath) {
    pwm_devpath(&g_pwmstate, DEFAULT_DEVPATH);
  }

  /* Mở thiết bị PWM */
  fd = open(g_pwmstate.devpath, O_RDONLY);
  if (fd < 0) {
    printf("pwm_main: open %s failed: %d\n", g_pwmstate.devpath, errno);
    return errno;
  }

  /* Cấu hình PWM */
  info.frequency = g_pwmstate.freq;
  info.duty = g_pwmstate.duty ? b16divi(uitoub16(g_pwmstate.duty), 100) : 0;

  printf("pwm_main: starting output with frequency: %" PRIu32 " duty: %08" PRIx32 "\n",
         info.frequency, (uint32_t)info.duty);

  ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
  if (ret < 0) {
    printf("pwm_main: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
    close(fd);
    return errno;
  }

  /* Bắt đầu PWM */
  ret = ioctl(fd, PWMIOC_START, 0);
  if (ret < 0) {
    printf("pwm_main: ioctl(PWMIOC_START) failed: %d\n", errno);
    close(fd);
    return errno;
  }

  /* Chờ trong khoảng thời gian duration */
  sleep(g_pwmstate.duration);

  /* Dừng PWM */
  printf("pwm_main: stopping output\n");
  ret = ioctl(fd, PWMIOC_STOP, 0);
  if (ret < 0) {
    printf("pwm_main: ioctl(PWMIOC_STOP) failed: %d\n", errno);
  }

  /* Đóng thiết bị */
  close(fd);
  fflush(stdout);
  return ret < 0 ? errno : 0;
}