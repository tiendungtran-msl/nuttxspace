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
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Common Macros */

#ifndef NULL
#ifdef __cplusplus
#define NULL           0
#else
#define NULL           ((void *) 0)
#endif
#endif

#define BMM150_TRUE                               1
#define BMM150_FALSE                              0

/* API Return Codes */

#define BMM150_OK                                 0
#define BMM150_E_NULL_PTR                         (-1)
#define BMM150_E_DEV_NOT_FOUND                    (-2)
#define BMM150_E_INVALID_CONFIG                   (-3)
#define BMM150_E_COM_FAIL                         (-4)

/* API Warning Codes */

#define BMM150_W_NORMAL_SELF_TEST_YZ_FAIL         1
#define BMM150_W_NORMAL_SELF_TEST_XZ_FAIL         2
#define BMM150_W_NORMAL_SELF_TEST_Z_FAIL          3
#define BMM150_W_NORMAL_SELF_TEST_XY_FAIL         4
#define BMM150_W_NORMAL_SELF_TEST_Y_FAIL          5
#define BMM150_W_NORMAL_SELF_TEST_X_FAIL          6
#define BMM150_W_NORMAL_SELF_TEST_XYZ_FAIL        7
#define BMM150_W_ADV_SELF_TEST_FAIL               8

/* I2C Address Options */

#define BMM150_DEFAULT_I2C_ADDRESS                0x10
#define BMM150_I2C_ADDRESS_CSB_LOW_SDO_HIGH       0x11
#define BMM150_I2C_ADDRESS_CSB_HIGH_SDO_LOW       0x12
#define BMM150_I2C_ADDRESS_CSB_HIGH_SDO_HIGH      0x13

/* Chip ID */

#define BMM150_CHIP_ID                            0x32

/* Soft Reset Value */

#define BMM150_SET_SOFT_RESET                     0x82

/* Register Map */

#define BMM150_REG_CHIP_ID                        0x40
#define BMM150_REG_DATA_X_LSB                     0x42
#define BMM150_REG_DATA_X_MSB                     0x43
#define BMM150_REG_DATA_Y_LSB                     0x44
#define BMM150_REG_DATA_Y_MSB                     0x45
#define BMM150_REG_DATA_Z_LSB                     0x46
#define BMM150_REG_DATA_Z_MSB                     0x47
#define BMM150_REG_RHALL_LSB                      0x48
#define BMM150_REG_RHALL_MSB                      0x49
#define BMM150_REG_INT_STATUS                     0x4A
#define BMM150_REG_POWER_CONTROL                  0x4B
#define BMM150_REG_OP_MODE                        0x4C
#define BMM150_REG_INT_CONFIG                     0x4D
#define BMM150_REG_AXES_ENABLE                    0x4E
#define BMM150_REG_LOW_THRESHOLD                  0x4F
#define BMM150_REG_HIGH_THRESHOLD                 0x50
#define BMM150_REG_REP_XY                         0x51
#define BMM150_REG_REP_Z                          0x52

/* Backward compatibility aliases */

#define BMM150_REG_DATA_READY_STATUS              BMM150_REG_RHALL_LSB
#define BMM150_REG_PWR_CTRL                       BMM150_REG_POWER_CONTROL
#define BMM150_REG_LOW_THRES                      BMM150_REG_LOW_THRESHOLD
#define BMM150_REG_HIGH_THRES                     BMM150_REG_HIGH_THRESHOLD

/* Trim/Calibration Registers */

#define BMM150_DIG_X1                             0x5D
#define BMM150_DIG_Y1                             0x5E
#define BMM150_DIG_Z4_LSB                         0x62
#define BMM150_DIG_Z4_MSB                         0x63
#define BMM150_DIG_X2                             0x64
#define BMM150_DIG_Y2                             0x65
#define BMM150_DIG_Z2_LSB                         0x68
#define BMM150_DIG_Z2_MSB                         0x69
#define BMM150_DIG_Z1_LSB                         0x6A
#define BMM150_DIG_Z1_MSB                         0x6B
#define BMM150_DIG_XYZ1_LSB                       0x6C
#define BMM150_DIG_XYZ1_MSB                       0x6D
#define BMM150_DIG_Z3_LSB                         0x6E
#define BMM150_DIG_Z3_MSB                         0x6F
#define BMM150_DIG_XY2                            0x70
#define BMM150_DIG_XY1                            0x71

/* Power Mode Definitions */

#define BMM150_POWERMODE_NORMAL                   0x00
#define BMM150_POWERMODE_FORCED                   0x01
#define BMM150_POWERMODE_SLEEP                    0x03
#define BMM150_POWERMODE_SUSPEND                  0x04

/* Power Control Settings */

#define BMM150_POWER_CNTRL_DISABLE                0x00
#define BMM150_POWER_CNTRL_ENABLE                 0x01

/* Operation Mode Register (0x4C) bits */

#define BMM150_OP_MODE_MSK                        0x06
#define BMM150_OP_MODE_POS                        0x01

/* Data Rate Definitions (ODR) */

#define BMM150_DATA_RATE_10HZ                     0x00
#define BMM150_DATA_RATE_02HZ                     0x01
#define BMM150_DATA_RATE_06HZ                     0x02
#define BMM150_DATA_RATE_08HZ                     0x03
#define BMM150_DATA_RATE_15HZ                     0x04
#define BMM150_DATA_RATE_20HZ                     0x05
#define BMM150_DATA_RATE_25HZ                     0x06
#define BMM150_DATA_RATE_30HZ                     0x07

#define BMM150_ODR_MAX                            0x07
#define BMM150_ODR_MSK                            0x38
#define BMM150_ODR_POS                            0x03

/* Alternative ODR naming */

#define BMM150_ODR_10HZ                           (0 << 3)
#define BMM150_ODR_2HZ                            (1 << 3)
#define BMM150_ODR_6HZ                            (2 << 3)
#define BMM150_ODR_8HZ                            (3 << 3)
#define BMM150_ODR_15HZ                           (4 << 3)
#define BMM150_ODR_20HZ                           (5 << 3)
#define BMM150_ODR_25HZ                           (6 << 3)
#define BMM150_ODR_30HZ                           (7 << 3)

/* Delay Time Settings (microseconds) */

#define BMM150_DELAY_SOFT_RESET                   1000
#define BMM150_DELAY_NORMAL_SELF_TEST             2000
#define BMM150_START_UP_TIME                      3000
#define BMM150_DELAY_ADV_SELF_TEST                4000

/* XYZ Channel Enable/Disable */

#define BMM150_XYZ_CHANNEL_ENABLE                 0x00
#define BMM150_XYZ_CHANNEL_DISABLE                0x07

/* Preset Modes */

#define BMM150_PRESETMODE_LOWPOWER                0x01
#define BMM150_PRESETMODE_REGULAR                 0x02
#define BMM150_PRESETMODE_HIGHACCURACY            0x03
#define BMM150_PRESETMODE_ENHANCED                0x04

/* Repetitions for XY-axis */

#define BMM150_REPXY_LOWPOWER                     0x01
#define BMM150_REPXY_REGULAR                      0x04
#define BMM150_REPXY_ENHANCED                     0x07
#define BMM150_REPXY_HIGHACCURACY                 0x17

/* Alternative naming */

#define BMM150_REP_XY_REGULAR                     0x04
#define BMM150_REP_XY_ENHANCED                    0x07
#define BMM150_REP_XY_HIGH_ACCURACY               0x17
#define BMM150_REP_XY_LOW_POWER                   0x01

/* Repetitions for Z-axis */

#define BMM150_REPZ_LOWPOWER                      0x01
#define BMM150_REPZ_REGULAR                       0x07
#define BMM150_REPZ_ENHANCED                      0x0D
#define BMM150_REPZ_HIGHACCURACY                  0x29

/* Alternative naming */

#define BMM150_REP_Z_REGULAR                      0x07
#define BMM150_REP_Z_ENHANCED                     0x0D
#define BMM150_REP_Z_HIGH_ACCURACY                0x29
#define BMM150_REP_Z_LOW_POWER                    0x01

/* Sensor Settings Selection Macros */

#define BMM150_SEL_DATA_RATE                      (1)
#define BMM150_SEL_CONTROL_MEASURE                (1 << 1)
#define BMM150_SEL_XY_REP                         (1 << 2)
#define BMM150_SEL_Z_REP                          (1 << 3)
#define BMM150_SEL_DRDY_PIN_EN                    (1 << 4)
#define BMM150_SEL_INT_PIN_EN                     (1 << 5)
#define BMM150_SEL_DRDY_POLARITY                  (1 << 6)
#define BMM150_SEL_INT_LATCH                      (1 << 7)
#define BMM150_SEL_INT_POLARITY                   (1 << 8)
#define BMM150_SEL_DATA_OVERRUN_INT               (1 << 9)
#define BMM150_SEL_OVERFLOW_INT                   (1 << 10)
#define BMM150_SEL_HIGH_THRESHOLD_INT             (1 << 11)
#define BMM150_SEL_LOW_THRESHOLD_INT              (1 << 12)
#define BMM150_SEL_LOW_THRESHOLD_SETTING          (1 << 13)
#define BMM150_SEL_HIGH_THRESHOLD_SETTING         (1 << 14)

/* Threshold Interrupt Settings */

#define BMM150_THRESHOLD_X                        0x06
#define BMM150_THRESHOLD_Y                        0x05
#define BMM150_THRESHOLD_Z                        0x03

#define BMM150_HIGH_THRESHOLD_INT_MSK             0x38
#define BMM150_HIGH_THRESHOLD_INT_POS             0x03
#define BMM150_LOW_THRESHOLD_INT_MSK              0x07

/* User Configurable Interrupt Settings */

#define BMM150_INT_ENABLE                         0x01
#define BMM150_INT_DISABLE                        0x00
#define BMM150_ACTIVE_HIGH_POLARITY               0x01
#define BMM150_ACTIVE_LOW_POLARITY                0x00
#define BMM150_LATCHED                            0x01
#define BMM150_NON_LATCHED                        0x00

/* Interrupt Status Bits */

#define BMM150_INT_THRESHOLD_X_LOW                (1 << 0)
#define BMM150_INT_THRESHOLD_Y_LOW                (1 << 1)
#define BMM150_INT_THRESHOLD_Z_LOW                (1 << 2)
#define BMM150_INT_THRESHOLD_X_HIGH               (1 << 3)
#define BMM150_INT_THRESHOLD_Y_HIGH               (1 << 4)
#define BMM150_INT_THRESHOLD_Z_HIGH               (1 << 5)
#define BMM150_INT_DATA_OVERFLOW                  (1 << 6)
#define BMM150_INT_DATA_OVERRUN                   (1 << 7)
#define BMM150_INT_DATA_READY                     (1 << 8)

/* Interrupt Asserted Macros */

#define BMM150_INT_ASSERTED_DRDY                  0x0100
#define BMM150_INT_ASSERTED_LOW_THRES             0x0007
#define BMM150_INT_ASSERTED_HIGH_THRES            0x0380

/* Interrupt Control Bit Masks and Positions */

#define BMM150_DRDY_EN_MSK                        0x80
#define BMM150_DRDY_EN_POS                        0x07
#define BMM150_DRDY_POLARITY_MSK                  0x04
#define BMM150_DRDY_POLARITY_POS                  0x02
#define BMM150_INT_PIN_EN_MSK                     0x40
#define BMM150_INT_PIN_EN_POS                     0x06
#define BMM150_INT_LATCH_MSK                      0x02
#define BMM150_INT_LATCH_POS                      0x01
#define BMM150_INT_POLARITY_MSK                   0x01
#define BMM150_DRDY_STATUS_MSK                    0x01

/* Power Control Bit Masks and Positions */

#define BMM150_PWR_CNTRL_MSK                      0x01
#define BMM150_CONTROL_MEASURE_MSK                0x38
#define BMM150_CONTROL_MEASURE_POS                0x03
#define BMM150_POWER_CONTROL_BIT_MSK              0x01
#define BMM150_POWER_CONTROL_BIT_POS              0x00

/* Data Bit Masks and Positions */

#define BMM150_DATA_X_MSK                         0xF8
#define BMM150_DATA_X_POS                         0x03

#define BMM150_DATA_Y_MSK                         0xF8
#define BMM150_DATA_Y_POS                         0x03

#define BMM150_DATA_Z_MSK                         0xFE
#define BMM150_DATA_Z_POS                         0x01

#define BMM150_DATA_RHALL_MSK                     0xFC
#define BMM150_DATA_RHALL_POS                     0x02

#define BMM150_DATA_OVERRUN_INT_MSK               0x80
#define BMM150_DATA_OVERRUN_INT_POS               0x07

#define BMM150_OVERFLOW_INT_MSK                   0x40
#define BMM150_OVERFLOW_INT_POS                   0x06

/* Overflow Definitions */

#define BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP        (-4096)
#define BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL         (-16384)
#define BMM150_OVERFLOW_OUTPUT                    (-32768)
#define BMM150_NEGATIVE_SATURATION_Z              (-32767)
#define BMM150_POSITIVE_SATURATION_Z              32767

/* Self-test Settings */

#define BMM150_DISABLE_XY_AXIS                    0x03
#define BMM150_SELF_TEST_REP_Z                    0x04

/* Self-test Selection */

#define BMM150_SELF_TEST_NORMAL                   0
#define BMM150_SELF_TEST_ADVANCED                 1

/* Advanced Self-test Current Settings */

#define BMM150_DISABLE_SELF_TEST_CURRENT          0x00
#define BMM150_ENABLE_NEGATIVE_CURRENT            0x02
#define BMM150_ENABLE_POSITIVE_CURRENT            0x03

/* Normal Self-test Status */

#define BMM150_SELF_TEST_STATUS_XYZ_FAIL          0x00
#define BMM150_SELF_TEST_STATUS_SUCCESS           0x07

#define BMM150_SELF_TEST_MSK                      0x01
#define BMM150_SELF_TEST_POS                      0x00
#define BMM150_ADV_SELF_TEST_MSK                  0xC0
#define BMM150_ADV_SELF_TEST_POS                  0x06

/* Register Read Lengths */

#define BMM150_LEN_SELF_TEST                      5
#define BMM150_LEN_SETTING_DATA                   8
#define BMM150_LEN_XYZR_DATA                      8

/* Boundary Check Macros */

#define BMM150_BOUNDARY_MAXIMUM                   0
#define BMM150_BOUNDARY_MINIMUM                   1

/* Macro to SET and GET BITS of a register */

#define BMM150_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))

#define BMM150_GET_BITS(reg_data, bitname) \
    ((reg_data & (bitname##_MSK)) >> (bitname##_POS))

#define BMM150_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | (data & bitname##_MSK))

#define BMM150_GET_BITS_POS_0(reg_data, bitname) \
    (reg_data & (bitname##_MSK))

#endif /* __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_BMM150_REGS_H */