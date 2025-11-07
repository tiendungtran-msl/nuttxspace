/****************************************************************************
 * make_pwm_main.c
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

#define DEVICE "/dev/pwm2"

struct pwm_info_s pwm2_info {
    .frequency = 10,
    .duty = 0x00ff,
    .cpol = PWM_CPOL_LOW,
    .dcpol = PWM_DCPOL_LOW,
 }

int main()
{
int fd = open("/dev/pwm2", O_RDWR);
if (fd < 0)
{
    printf("Fail to open PWW: %d\n", errno);
    return -1;
}

if (ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)&pwm2_info) < 0) {
    printf("Failed to set characteristics of pwm2: %d\n", errno);
    close(fd);
    return -1;
}

if (ioctl(fd, PWMIOC_START, 0) < 0) {
    printf("Failed to start pwm2: %d\n", errno);
    close(fd);
    return -1;
}

sleep(10);

if (ioctl(fd, PWMIOC_STOP, 0) < 0) {
    printf("Failed to stop PWM: %d\n", errno);
    close(fd);
    return -1;
}
}