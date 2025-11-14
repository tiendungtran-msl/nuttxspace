/****************************************************************************
 * apps/examples/imu_system/drivers/bmm150/bmm150_regs.h
 *
 * BMM150 Magnetometer Register Definitions (I2C Interface)
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_IMU_SYSTEM_BMM150_REGS_H
#define __APPS_EXAMPLES_IMU_SYSTEM_BMM150_REGS_H

/* I2C Address */
#define BMM150_I2C_ADDR_PRIMARY    0x10  /* SDO = GND */
#define BMM150_I2C_ADDR_SECONDARY  0x12  /* SDO = VDDIO */

/* Register Map */
#define BMM150_CHIP_ID             0x40
#define BMM150_DATA_X_LSB          0x42
#define BMM150_DATA_X_MSB          0x43
#define BMM150_DATA_Y_LSB          0x44
#define BMM150_DATA_Y_MSB          0x45
#define BMM150_DATA_Z_LSB          0x46
#define BMM150_DATA_Z_MSB          0x47
#define BMM150_RHALL_LSB           0x48
#define BMM150_RHALL_MSB           0x49
#define BMM150_INT_STATUS          0x4A
#define BMM150_PWR_CTRL            0x4B
#define BMM150_OP_MODE             0x4C
#define BMM150_INT_CONFIG          0x4D
#define BMM150_AXES_ENABLE         0x4E
#define BMM150_LOW_THRES           0x4F
#define BMM150_HIGH_THRES          0x50
#define BMM150_REP_XY              0x51
#define BMM150_REP_Z               0x52

/* Trim Registers */
#define BMM150_DIG_X1              0x5D
#define BMM150_DIG_Y1              0x5E
#define BMM150_DIG_Z4_LSB          0x62
#define BMM150_DIG_Z4_MSB          0x63
#define BMM150_DIG_X2              0x64
#define BMM150_DIG_Y2              0x65
#define BMM150_DIG_Z2_LSB          0x68
#define BMM150_DIG_Z2_MSB          0x69
#define BMM150_DIG_Z1_LSB          0x6A
#define BMM150_DIG_Z1_MSB          0x6B
#define BMM150_DIG_XYZ1_LSB        0x6C
#define BMM150_DIG_XYZ1_MSB        0x6D
#define BMM150_DIG_Z3_LSB          0x6E
#define BMM150_DIG_Z3_MSB          0x6F
#define BMM150_DIG_XY2             0x70
#define BMM150_DIG_XY1             0x71

/* Chip ID Value */
#define BMM150_CHIP_ID_VALUE       0x32

/* Power Control */
#define BMM150_PWR_CTRL_POWER_ON   0x01
#define BMM150_PWR_CTRL_SOFT_RESET 0x82

/* Operation Mode */
#define BMM150_OP_MODE_NORMAL      0x00
#define BMM150_OP_MODE_FORCED      0x02
#define BMM150_OP_MODE_SLEEP       0x06
#define BMM150_OP_MODE_SUSPEND     0x18

/* Data Rate (in OP_MODE register) */
#define BMM150_ODR_10HZ            0x00
#define BMM150_ODR_2HZ             0x08
#define BMM150_ODR_6HZ             0x10
#define BMM150_ODR_8HZ             0x18
#define BMM150_ODR_15HZ            0x20
#define BMM150_ODR_20HZ            0x28
#define BMM150_ODR_25HZ            0x30
#define BMM150_ODR_30HZ            0x38

/* Preset Modes - Repetitions */
#define BMM150_LOWPOWER_REPXY      1
#define BMM150_LOWPOWER_REPZ       2
#define BMM150_REGULAR_REPXY       4
#define BMM150_REGULAR_REPZ        14
#define BMM150_HIGHACCURACY_REPXY  23
#define BMM150_HIGHACCURACY_REPZ   82

/* Data ready bit */
#define BMM150_DATA_READY          0x01

#endif /* __APPS_EXAMPLES_IMU_SYSTEM_BMM150_REGS_H */