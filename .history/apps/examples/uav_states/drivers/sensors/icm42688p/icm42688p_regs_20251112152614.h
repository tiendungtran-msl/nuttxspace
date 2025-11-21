/****************************************************************************
 * apps/examples/imu_system/drivers/icm42688p/icm42688p_regs.h
 *
 * ICM-42688-P Register Definitions
 * Adapted from PX4 Autopilot for NuttX
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_ICM42688P_REGS_H
#define __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_ICM42688P_REGS_H

#include <stdint.h>

/* Bank Selection */
#define ICM42688P_REG_BANK_SEL              0x76

typedef enum {
    BANK_0 = 0,
    BANK_1 = 1,
    BANK_2 = 2,
    BANK_4 = 4
} icm42688p_bank_t;

/* Bank 0 Registers */
#define ICM42688P_DEVICE_CONFIG             0x11
#define ICM42688P_DRIVE_CONFIG              0x13
#define ICM42688P_INT_CONFIG                0x14
#define ICM42688P_FIFO_CONFIG               0x16
#define ICM42688P_TEMP_DATA1                0x1D
#define ICM42688P_TEMP_DATA0                0x1E
#define ICM42688P_ACCEL_DATA_X1             0x1F
#define ICM42688P_ACCEL_DATA_X0             0x20
#define ICM42688P_ACCEL_DATA_Y1             0x21
#define ICM42688P_ACCEL_DATA_Y0             0x22
#define ICM42688P_ACCEL_DATA_Z1             0x23
#define ICM42688P_ACCEL_DATA_Z0             0x24
#define ICM42688P_GYRO_DATA_X1              0x25
#define ICM42688P_GYRO_DATA_X0              0x26
#define ICM42688P_GYRO_DATA_Y1              0x27
#define ICM42688P_GYRO_DATA_Y0              0x28
#define ICM42688P_GYRO_DATA_Z1              0x29
#define ICM42688P_GYRO_DATA_Z0              0x2A
#define ICM42688P_TMST_FSYNCH               0x2B
#define ICM42688P_TMST_FSYNCL               0x2C
#define ICM42688P_INT_STATUS                0x2D
#define ICM42688P_FIFO_COUNTH               0x2E
#define ICM42688P_FIFO_COUNTL               0x2F
#define ICM42688P_FIFO_DATA                 0x30
#define ICM42688P_APEX_DATA0                0x31
#define ICM42688P_SIGNAL_PATH_RESET         0x4B
#define ICM42688P_INTF_CONFIG0              0x4C
#define ICM42688P_INTF_CONFIG1              0x4D
#define ICM42688P_PWR_MGMT0                 0x4E
#define ICM42688P_GYRO_CONFIG0              0x4F
#define ICM42688P_ACCEL_CONFIG0             0x50
#define ICM42688P_GYRO_CONFIG1              0x51
#define ICM42688P_GYRO_ACCEL_CONFIG0        0x52
#define ICM42688P_ACCEL_CONFIG1             0x53
#define ICM42688P_TMST_CONFIG               0x54
#define ICM42688P_FIFO_CONFIG1              0x5F
#define ICM42688P_FIFO_CONFIG2              0x60
#define ICM42688P_FIFO_CONFIG3              0x61
#define ICM42688P_FSYNC_CONFIG              0x62
#define ICM42688P_INT_CONFIG0               0x63
#define ICM42688P_INT_CONFIG1               0x64
#define ICM42688P_INT_SOURCE0               0x65
#define ICM42688P_INT_SOURCE1               0x66
#define ICM42688P_WHO_AM_I                  0x75
#define ICM42688P_REG_BANK_SEL              0x76

/* WHO_AM_I Value */
#define ICM42688P_WHO_AM_I_VALUE            0x47

/* PWR_MGMT0 Bits */
#define PWR_MGMT0_TEMP_DIS                  (1 << 5)
#define PWR_MGMT0_IDLE                      (1 << 4)
#define PWR_MGMT0_GYRO_MODE_OFF             (0 << 2)
#define PWR_MGMT0_GYRO_MODE_STANDBY         (1 << 2)
#define PWR_MGMT0_GYRO_MODE_LN              (3 << 2)
#define PWR_MGMT0_ACCEL_MODE_OFF            (0 << 0)
#define PWR_MGMT0_ACCEL_MODE_LP             (2 << 0)
#define PWR_MGMT0_ACCEL_MODE_LN             (3 << 0)

/* GYRO_CONFIG0 Bits */
#define GYRO_CONFIG0_FS_SEL_2000DPS         (0 << 5)
#define GYRO_CONFIG0_FS_SEL_1000DPS         (1 << 5)
#define GYRO_CONFIG0_FS_SEL_500DPS          (2 << 5)
#define GYRO_CONFIG0_FS_SEL_250DPS          (3 << 5)
#define GYRO_CONFIG0_ODR_32KHZ              0x01
#define GYRO_CONFIG0_ODR_16KHZ              0x02
#define GYRO_CONFIG0_ODR_8KHZ               0x03
#define GYRO_CONFIG0_ODR_4KHZ               0x04
#define GYRO_CONFIG0_ODR_2KHZ               0x05
#define GYRO_CONFIG0_ODR_1KHZ               0x06

/* ACCEL_CONFIG0 Bits */
#define ACCEL_CONFIG0_FS_SEL_16G            (0 << 5)
#define ACCEL_CONFIG0_FS_SEL_8G             (1 << 5)
#define ACCEL_CONFIG0_FS_SEL_4G             (2 << 5)
#define ACCEL_CONFIG0_FS_SEL_2G             (3 << 5)
#define ACCEL_CONFIG0_ODR_32KHZ             0x01
#define ACCEL_CONFIG0_ODR_16KHZ             0x02
#define ACCEL_CONFIG0_ODR_8KHZ              0x03
#define ACCEL_CONFIG0_ODR_1KHZ              0x06

/* FIFO Configuration */
#define FIFO_CONFIG_MODE_BYPASS             (0 << 6)
#define FIFO_CONFIG_MODE_STREAM             (1 << 6)
#define FIFO_CONFIG_MODE_STOP_FULL          (2 << 6)

/* INT_CONFIG Bits */
#define INT_CONFIG_INT1_MODE_PULSED         (0 << 2)
#define INT_CONFIG_INT1_MODE_LATCHED        (1 << 2)
#define INT_CONFIG_INT1_DRIVE_PP            (0 << 1)
#define INT_CONFIG_INT1_DRIVE_OD            (1 << 1)
#define INT_CONFIG_INT1_POLARITY_LOW        (0 << 0)
#define INT_CONFIG_INT1_POLARITY_HIGH       (1 << 0)

/* INT_SOURCE0 Bits */
#define INT_SOURCE0_UI_DRDY_INT1_EN         (1 << 3)
#define INT_SOURCE0_FIFO_THS_INT1_EN        (1 << 2)
#define INT_SOURCE0_FIFO_FULL_INT1_EN       (1 << 1)

/* FIFO Packet Structure */
#define FIFO_PACKET_SIZE                    16
#define FIFO_MAX_SIZE                       2048

/* Sensitivity Values */
#define GYRO_SENSITIVITY_2000DPS            16.4f
#define GYRO_SENSITIVITY_1000DPS            32.8f
#define GYRO_SENSITIVITY_500DPS             65.5f
#define GYRO_SENSITIVITY_250DPS             131.0f

#define ACCEL_SENSITIVITY_16G               2048.0f
#define ACCEL_SENSITIVITY_8G                4096.0f
#define ACCEL_SENSITIVITY_4G                8192.0f
#define ACCEL_SENSITIVITY_2G                16384.0f

#define TEMP_SENSITIVITY                    132.48f
#define TEMP_OFFSET                         25.0f

#endif /* __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_ICM42688P_REGS_H */