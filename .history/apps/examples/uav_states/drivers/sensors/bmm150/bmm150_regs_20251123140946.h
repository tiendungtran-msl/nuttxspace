/****************************************************************************
 * apps/examples/uav_states/drivers/sensors/bmm150/bmm150_regs.h
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

#ifndef __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_BMM150_REGS_H
#define __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_BMM150_REGS_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C Address */

#define BMM150_I2C_ADDR             0x32

/* Register Map */

#define BMM150_REG_CHIP_ID          0x40
#define BMM150_REG_DATA_X_LSB       0x42
#define BMM150_REG_DATA_X_MSB       0x43
#define BMM150_REG_DATA_Y_LSB       0x44
#define BMM150_REG_DATA_Y_MSB       0x45
#define BMM150_REG_DATA_Z_LSB       0x46
#define BMM150_REG_DATA_Z_MSB       0x47
#define BMM150_REG_RHALL_LSB        0x48
#define BMM150_REG_RHALL_MSB        0x49
#define BMM150_REG_INT_STATUS       0x4A
#define BMM150_REG_PWR_CTRL         0x4B
#define BMM150_REG_OP_MODE          0x4C
#define BMM150_REG_INT_CONFIG       0x4D
#define BMM150_REG_AXES_ENABLE      0x4E
#define BMM150_REG_LOW_THRES        0x4F
#define BMM150_REG_HIGH_THRES       0x50
#define BMM150_REG_REP_XY           0x51
#define BMM150_REG_REP_Z            0x52

/* Trim/Calibration Registers (NVM - Non-Volatile Memory) */

#define BMM150_REG_DIG_X1           0x5D
#define BMM150_REG_DIG_Y1           0x5E
#define BMM150_REG_DIG_Z4_LSB       0x62
#define BMM150_REG_DIG_Z4_MSB       0x63
#define BMM150_REG_DIG_X2           0x64
#define BMM150_REG_DIG_Y2           0x65
#define BMM150_REG_DIG_Z2_LSB       0x68
#define BMM150_REG_DIG_Z2_MSB       0x69
#define BMM150_REG_DIG_Z1_LSB       0x6A
#define BMM150_REG_DIG_Z1_MSB       0x6B
#define BMM150_REG_DIG_XYZ1_LSB     0x6C
#define BMM150_REG_DIG_XYZ1_MSB     0x6D
#define BMM150_REG_DIG_Z3_LSB       0x6E
#define BMM150_REG_DIG_Z3_MSB       0x6F
#define BMM150_REG_DIG_XY2          0x70
#define BMM150_REG_DIG_XY1          0x71

/* Chip ID Value */

#define BMM150_CHIP_ID_VALUE        0x32

/* Power Control Register (0x4B) */

#define BMM150_PWR_CTRL_POWER_ON    0x01
#define BMM150_PWR_CTRL_POWER_OFF   0x00
#define BMM150_PWR_CTRL_SOFT_RESET  0x82

/* Operation Mode Register (0x4C) */

#define BMM150_OP_MODE_NORMAL       0x00
#define BMM150_OP_MODE_FORCED       0x02
#define BMM150_OP_MODE_SLEEP        0x06
#define BMM150_OP_MODE_SUSPEND      0x18

/* Data Rate (bits [5:3] in OP_MODE register) */

#define BMM150_ODR_10HZ             (0 << 3)  /* 10 Hz */
#define BMM150_ODR_2HZ              (1 << 3)  /* 2 Hz */
#define BMM150_ODR_6HZ              (2 << 3)  /* 6 Hz */
#define BMM150_ODR_8HZ              (3 << 3)  /* 8 Hz */
#define BMM150_ODR_15HZ             (4 << 3)  /* 15 Hz */
#define BMM150_ODR_20HZ             (5 << 3)  /* 20 Hz */
#define BMM150_ODR_25HZ             (6 << 3)  /* 25 Hz */
#define BMM150_ODR_30HZ             (7 << 3)  /* 30 Hz */

/* Repetitions for XY-axis (0x51) */

#define BMM150_REP_XY_REGULAR       0x04  /* Regular preset */
#define BMM150_REP_XY_ENHANCED      0x07  /* Enhanced preset */
#define BMM150_REP_XY_HIGH_ACCURACY 0x17  /* High accuracy preset */
#define BMM150_REP_XY_LOW_POWER     0x03  /* Low power preset */

/* Repetitions for Z-axis (0x52) */

#define BMM150_REP_Z_REGULAR        0x0E  /* Regular preset */
#define BMM150_REP_Z_ENHANCED       0x1A  /* Enhanced preset */
#define BMM150_REP_Z_HIGH_ACCURACY  0x52  /* High accuracy preset */
#define BMM150_REP_Z_LOW_POWER      0x03  /* Low power preset */

/* Data bits masks */

#define BMM150_DATA_X_MASK          0xFFF8
#define BMM150_DATA_Y_MASK          0xFFF8
#define BMM150_DATA_Z_MASK          0xFFFE
#define BMM150_DATA_RHALL_MASK      0xFFFC

/* Self-test masks */

#define BMM150_SELF_TEST_MASK       0x01

/* Overflow bit */

#define BMM150_OVERFLOW_BIT         0x80

#endif /* __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_BMM150_REGS_H */