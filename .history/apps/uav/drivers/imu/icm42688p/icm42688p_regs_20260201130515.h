/****************************************************************************
 * apps/uav/drivers/imu/icm42688p/icm42688p_regs.h
 *
 * Định nghĩa các thanh ghi cho ICM-42688-P IMU
 * 6-DOF: 3-axis Gyroscope + 3-axis Accelerometer
 ****************************************************************************/

#ifndef __APPS_UAV_DRIVERS_IMU_ICM42688P_REGS_H
#define __APPS_UAV_DRIVERS_IMU_ICM42688P_REGS_H

#include <stdint.h>

/* Chọn Bank Register */
#define ICM42688P_REG_BANK_SEL              0x76

/* Bank Selection */
typedef enum
{
  BANK_0 = 0,
  BANK_1 = 1,
  BANK_2 = 2,
  BANK_4 = 4
} icm42688p_bank_t;

/* ========== BANK 0 Registers ========== */

#define ICM42688P_DEVICE_CONFIG             0x11  /* Cấu hình thiết bị */
#define ICM42688P_DRIVE_CONFIG              0x13  /* Cấu hình drive strength */
#define ICM42688P_INT_CONFIG                0x14  /* Cấu hình interrupt */
#define ICM42688P_FIFO_CONFIG               0x16  /* Cấu hình FIFO */

/* Thanh ghi dữ liệu (read-only) */
#define ICM42688P_TEMP_DATA1                0x1D  /* Temperature MSB */
#define ICM42688P_TEMP_DATA0                0x1E  /* Temperature LSB */
#define ICM42688P_ACCEL_DATA_X1             0x1F  /* Accel X MSB */
#define ICM42688P_ACCEL_DATA_X0             0x20  /* Accel X LSB */
#define ICM42688P_ACCEL_DATA_Y1             0x21  /* Accel Y MSB */
#define ICM42688P_ACCEL_DATA_Y0             0x22  /* Accel Y LSB */
#define ICM42688P_ACCEL_DATA_Z1             0x23  /* Accel Z MSB */
#define ICM42688P_ACCEL_DATA_Z0             0x24  /* Accel Z LSB */
#define ICM42688P_GYRO_DATA_X1              0x25  /* Gyro X MSB */
#define ICM42688P_GYRO_DATA_X0              0x26  /* Gyro X LSB */
#define ICM42688P_GYRO_DATA_Y1              0x27  /* Gyro Y MSB */
#define ICM42688P_GYRO_DATA_Y0              0x28  /* Gyro Y LSB */
#define ICM42688P_GYRO_DATA_Z1              0x29  /* Gyro Z MSB */
#define ICM42688P_GYRO_DATA_Z0              0x2A  /* Gyro Z LSB */

/* Thanh ghi trạng thái */
#define ICM42688P_TMST_FSYNCH               0x2B  /* Timestamp FSYNC High */
#define ICM42688P_TMST_FSYNCL               0x2C  /* Timestamp FSYNC Low */
#define ICM42688P_INT_STATUS                0x2D  /* Interrupt status */
#define ICM42688P_FIFO_COUNTH               0x2E  /* FIFO count MSB */
#define ICM42688P_FIFO_COUNTL               0x2F  /* FIFO count LSB */
#define ICM42688P_FIFO_DATA                 0x30  /* FIFO data */
#define ICM42688P_APEX_DATA0                0x31  /* APEX data */

/* Thanh ghi điều khiển */
#define ICM42688P_SIGNAL_PATH_RESET         0x4B  /* Reset signal path */
#define ICM42688P_INTF_CONFIG0              0x4C  /* Interface config 0 */
#define ICM42688P_INTF_CONFIG1              0x4D  /* Interface config 1 */
#define ICM42688P_PWR_MGMT0                 0x4E  /* Power management */
#define ICM42688P_GYRO_CONFIG0              0x4F  /* Gyro config */
#define ICM42688P_ACCEL_CONFIG0             0x50  /* Accel config */
#define ICM42688P_GYRO_CONFIG1              0x51  /* Gyro config 1 */
#define ICM42688P_GYRO_ACCEL_CONFIG0        0x52  /* Gyro+Accel config */
#define ICM42688P_ACCEL_CONFIG1             0x53  /* Accel config 1 */
#define ICM42688P_TMST_CONFIG               0x54  /* Timestamp config */
#define ICM42688P_FIFO_CONFIG1              0x5F  /* FIFO config 1 */
#define ICM42688P_FIFO_CONFIG2              0x60  /* FIFO config 2 */
#define ICM42688P_FIFO_CONFIG3              0x61  /* FIFO config 3 */
#define ICM42688P_FSYNC_CONFIG              0x62  /* FSYNC config */
#define ICM42688P_INT_CONFIG0               0x63  /* Interrupt config 0 */
#define ICM42688P_INT_CONFIG1               0x64  /* Interrupt config 1 */
#define ICM42688P_INT_SOURCE0               0x65  /* Interrupt source 0 */
#define ICM42688P_INT_SOURCE1               0x66  /* Interrupt source 1 */
#define ICM42688P_WHO_AM_I                  0x75  /* Device ID */

/* WHO_AM_I Value */
#define ICM42688P_WHO_AM_I_VALUE            0x47

/* ========== PWR_MGMT0 Register Bits ========== */
#define PWR_MGMT0_TEMP_DIS                  (1 << 5)  /* Disable temperature */
#define PWR_MGMT0_IDLE                      (1 << 4)  /* Idle mode */
#define PWR_MGMT0_GYRO_MODE_OFF             (0 << 2)  /* Gyro OFF */
#define PWR_MGMT0_GYRO_MODE_STANDBY         (1 << 2)  /* Gyro Standby */
#define PWR_MGMT0_GYRO_MODE_LN              (3 << 2)  /* Gyro Low Noise */
#define PWR_MGMT0_ACCEL_MODE_OFF            (0 << 0)  /* Accel OFF */
#define PWR_MGMT0_ACCEL_MODE_LP             (2 << 0)  /* Accel Low Power */
#define PWR_MGMT0_ACCEL_MODE_LN             (3 << 0)  /* Accel Low Noise */

/* ========== GYRO_CONFIG0 Register ========== */
/* Full Scale Range (FS_SEL) */
#define GYRO_CONFIG0_FS_SEL_2000DPS         (0 << 5)  /* ±2000 dps */
#define GYRO_CONFIG0_FS_SEL_1000DPS         (1 << 5)  /* ±1000 dps */
#define GYRO_CONFIG0_FS_SEL_500DPS          (2 << 5)  /* ±500 dps */
#define GYRO_CONFIG0_FS_SEL_250DPS          (3 << 5)  /* ±250 dps */

/* Output Data Rate (ODR) */
#define GYRO_CONFIG0_ODR_32KHZ              0x01      /* 32 kHz */
#define GYRO_CONFIG0_ODR_16KHZ              0x02      /* 16 kHz */
#define GYRO_CONFIG0_ODR_8KHZ               0x03      /* 8 kHz */
#define GYRO_CONFIG0_ODR_4KHZ               0x04      /* 4 kHz */
#define GYRO_CONFIG0_ODR_2KHZ               0x05      /* 2 kHz */
#define GYRO_CONFIG0_ODR_1KHZ               0x06      /* 1 kHz */
#define GYRO_CONFIG0_ODR_500HZ              0x0F      /* 500 Hz */
#define GYRO_CONFIG0_ODR_200HZ              0x07      /* 200 Hz */
#define GYRO_CONFIG0_ODR_100HZ              0x08      /* 100 Hz */

/* ========== ACCEL_CONFIG0 Register ========== */
/* Full Scale Range (FS_SEL) */
#define ACCEL_CONFIG0_FS_SEL_16G            (0 << 5)  /* ±16g */
#define ACCEL_CONFIG0_FS_SEL_8G             (1 << 5)  /* ±8g */
#define ACCEL_CONFIG0_FS_SEL_4G             (2 << 5)  /* ±4g */
#define ACCEL_CONFIG0_FS_SEL_2G             (3 << 5)  /* ±2g */

/* Output Data Rate (ODR) */
#define ACCEL_CONFIG0_ODR_32KHZ             0x01      /* 32 kHz */
#define ACCEL_CONFIG0_ODR_16KHZ             0x02      /* 16 kHz */
#define ACCEL_CONFIG0_ODR_8KHZ              0x03      /* 8 kHz */
#define ACCEL_CONFIG0_ODR_4KHZ              0x04      /* 4 kHz */
#define ACCEL_CONFIG0_ODR_2KHZ              0x05      /* 2 kHz */
#define ACCEL_CONFIG0_ODR_1KHZ              0x06      /* 1 kHz */
#define ACCEL_CONFIG0_ODR_500HZ             0x0F      /* 500 Hz */
#define ACCEL_CONFIG0_ODR_200HZ             0x07      /* 200 Hz */
#define ACCEL_CONFIG0_ODR_100HZ             0x08      /* 100 Hz */

/* ========== FIFO Configuration ========== */
#define FIFO_CONFIG_MODE_BYPASS             (0 << 6)  /* Bypass mode */
#define FIFO_CONFIG_MODE_STREAM             (1 << 6)  /* Stream mode */
#define FIFO_CONFIG_MODE_STOP_FULL          (2 << 6)  /* Stop when full */

/* ========== INT_CONFIG Bits ========== */
#define INT_CONFIG_INT1_MODE_PULSED         (0 << 2)  /* Pulsed interrupt */
#define INT_CONFIG_INT1_MODE_LATCHED        (1 << 2)  /* Latched interrupt */
#define INT_CONFIG_INT1_DRIVE_PP            (0 << 1)  /* Push-pull */
#define INT_CONFIG_INT1_DRIVE_OD            (1 << 1)  /* Open-drain */
#define INT_CONFIG_INT1_POLARITY_LOW        (0 << 0)  /* Active low */
#define INT_CONFIG_INT1_POLARITY_HIGH       (1 << 0)  /* Active high */

/* ========== INT_SOURCE0 Bits ========== */
#define INT_SOURCE0_UI_DRDY_INT1_EN         (1 << 3)  /* Data ready INT */
#define INT_SOURCE0_FIFO_THS_INT1_EN        (1 << 2)  /* FIFO threshold INT */
#define INT_SOURCE0_FIFO_FULL_INT1_EN       (1 << 1)  /* FIFO full INT */

/* ========== FIFO Packet Structure ========== */
#define FIFO_PACKET_SIZE                    16        /* Bytes per packet */
#define FIFO_MAX_SIZE                       2048      /* Maximum FIFO size */

/* ========== Sensitivity Values (LSB per unit) ========== */
/* Gyroscope Sensitivity (LSB/dps) */
#define GYRO_SENSITIVITY_2000DPS            16.4f
#define GYRO_SENSITIVITY_1000DPS            32.8f
#define GYRO_SENSITIVITY_500DPS             65.5f
#define GYRO_SENSITIVITY_250DPS             131.0f

/* Accelerometer Sensitivity (LSB/g) */
#define ACCEL_SENSITIVITY_16G               2048.0f
#define ACCEL_SENSITIVITY_8G                4096.0f
#define ACCEL_SENSITIVITY_4G                8192.0f
#define ACCEL_SENSITIVITY_2G                16384.0f

/* Temperature Sensitivity */
#define TEMP_SENSITIVITY                    132.48f   /* LSB/°C */
#define TEMP_OFFSET                         25.0f     /* °C offset */

#endif /* __APPS_UAV_DRIVERS_IMU_ICM42688P_REGS_H */
