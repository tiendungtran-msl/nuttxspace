/****************************************************************************
 * apps/examples/imu_system/drivers/bmm150/bmm150_regs.h
 *
 * BMM150 Magnetometer Register Definitions
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_BMM150_REGS_H
#define __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_BMM150_REGS_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Map */

#define BMM150_REG_CHIP_ID           0x40
#define BMM150_REG_DATA_X_LSB        0x42
#define BMM150_REG_DATA_X_MSB        0x43
#define BMM150_REG_DATA_Y_LSB        0x44
#define BMM150_REG_DATA_Y_MSB        0x45
#define BMM150_REG_DATA_Z_LSB        0x46
#define BMM150_REG_DATA_Z_MSB        0x47
#define BMM150_REG_RHALL_LSB         0x48
#define BMM150_REG_RHALL_MSB         0x49
#define BMM150_REG_INT_STATUS        0x4A
#define BMM150_REG_PWR_CTRL          0x4B
#define BMM150_REG_OP_MODE           0x4C
#define BMM150_REG_INT_CONFIG        0x4D
#define BMM150_REG_AXES_ENABLE       0x4E
#define BMM150_REG_LOW_THRES         0x4F
#define BMM150_REG_HIGH_THRES        0x50
#define BMM150_REG_REP_XY            0x51
#define BMM150_REG_REP_Z             0x52

/* Trim Registers */

#define BMM150_REG_DIG_X1            0x5D
#define BMM150_REG_DIG_Y1            0x5E
#define BMM150_REG_DIG_Z4_LSB        0x62
#define BMM150_REG_DIG_Z4_MSB        0x63
#define BMM150_REG_DIG_X2            0x64
#define BMM150_REG_DIG_Y2            0x65
#define BMM150_REG_DIG_Z2_LSB        0x68
#define BMM150_REG_DIG_Z2_MSB        0x69
#define BMM150_REG_DIG_Z1_LSB        0x6A
#define BMM150_REG_DIG_Z1_MSB        0x6B
#define BMM150_REG_DIG_XYZ1_LSB      0x6C
#define BMM150_REG_DIG_XYZ1_MSB      0x6D
#define BMM150_REG_DIG_Z3_LSB        0x6E
#define BMM150_REG_DIG_Z3_MSB        0x6F
#define BMM150_REG_DIG_XY2           0x70
#define BMM150_REG_DIG_XY1           0x71

/* Chip ID */

#define BMM150_CHIP_ID               0x32

/* Power Control */

#define BMM150_PWR_CTRL_POWER_ON     0x01
#define BMM150_PWR_CTRL_POWER_OFF    0x00

/* Operation Mode */

#define BMM150_OP_MODE_NORMAL        0x00
#define BMM150_OP_MODE_FORCED        0x01
#define BMM150_OP_MODE_SLEEP         0x03
#define BMM150_OP_MODE_MASK          0x06

#define BMM150_DATA_RATE_10HZ        (0 << 3)
#define BMM150_DATA_RATE_2HZ         (1 << 3)
#define BMM150_DATA_RATE_6HZ         (2 << 3)
#define BMM150_DATA_RATE_8HZ         (3 << 3)
#define BMM150_DATA_RATE_15HZ        (4 << 3)
#define BMM150_DATA_RATE_20HZ        (5 << 3)
#define BMM150_DATA_RATE_25HZ        (6 << 3)
#define BMM150_DATA_RATE_30HZ        (7 << 3)

/* Self-test */

#define BMM150_SELF_TEST_ENABLE      0x01

/* Repetition Settings */

#define BMM150_REP_XY_REGULAR        0x04
#define BMM150_REP_Z_REGULAR         0x0E
#define BMM150_REP_XY_ENHANCED       0x07
#define BMM150_REP_Z_ENHANCED        0x1A
#define BMM150_REP_XY_HIGH_ACCURACY  0x17
#define BMM150_REP_Z_HIGH_ACCURACY   0x52

#endif /* __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_BMM150_REGS_H */
