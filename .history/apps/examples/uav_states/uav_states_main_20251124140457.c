/****************************************************************************
 * apps/examples/uav_states/uav_states_main.c
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
#include <unistd.h>
#include "../tests/icm42688p_test.h"
#include "../tests/bmm150_test.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * uav_states_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
    printf("UAV States Application Starting...\n");
    fflush(stdout);

    usleep(1000000);  // Chờ 1 giây để đảm bảo mọi thứ ổn định
    
    /* Call the ICM42688P test function */
    
    bmm150_test_main(argc, argv);
}
