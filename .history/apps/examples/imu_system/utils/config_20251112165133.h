/****************************************************************************
 * apps/examples/imu_system/utils/config.h
 *
 * IMU System Configuration
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_IMU_SYSTEM_UTILS_CONFIG_H
#define __APPS_EXAMPLES_IMU_SYSTEM_UTILS_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Hardware Configuration */

#define IMU_NUM_ICM42688P           4
#define IMU_BMM150_DEVID            4
#define IMU_SPI_BUS                 1
#define IMU_SPI_FREQUENCY           10000000  /* 10 MHz */

/* Sensor Configuration */

#define IMU_ACCEL_FS_G              16        /* ±16g */
#define IMU_GYRO_FS_DPS             2000      /* ±2000 dps */
#define IMU_SENSOR_RATE_HZ          1000      /* 1 kHz sensor rate */
#define IMU_FUSION_RATE_HZ          100       /* 100 Hz fusion rate */
#define IMU_LED_RATE_HZ             10        /* 10 Hz LED update rate */

/* I2C Configuration */
#define IMU_I2C_BUS               1
#define IMU_I2C_FREQUENCY         400000    /* 400 kHz */

/* BMM150 Magnetometer Configuration */
#define IMU_BMM150_I2C_BUS        1         /* I2C1 */
#define IMU_BMM150_I2C_ADDR       BMM150_I2C_ADDR_PRIMARY  /* 0x10 */

/* Task Configuration */

#define IMU_SENSOR_TASK_PRIORITY    150
#define IMU_FUSION_TASK_PRIORITY    100
#define IMU_LED_TASK_PRIORITY       50

#define IMU_SENSOR_TASK_STACKSIZE   4096
#define IMU_FUSION_TASK_STACKSIZE   8192
#define IMU_LED_TASK_STACKSIZE      2048

/* Queue Configuration */

#define IMU_SENSOR_QUEUE_SIZE       64
#define IMU_FUSION_QUEUE_SIZE       32

/* LED Patterns */

#define IMU_LED_PATTERN_OFF         0
#define IMU_LED_PATTERN_SOLID       1
#define IMU_LED_PATTERN_SLOW_BLINK  2  /* Calibration */
#define IMU_LED_PATTERN_FAST_BLINK  3  /* Error */

#define IMU_LED_SLOW_BLINK_MS       1000
#define IMU_LED_FAST_BLINK_MS       200

/* Sensor Fusion Configuration */

#define IMU_MADGWICK_BETA           0.1f
#define IMU_MADGWICK_SAMPLE_FREQ    ((float)IMU_FUSION_RATE_HZ)

/* Calibration */

#define IMU_GYRO_CALIB_SAMPLES      1000
#define IMU_MAG_CALIB_SAMPLES       500

/* Debug Configuration */

#ifdef CONFIG_EXAMPLES_IMU_SYSTEM_DEBUG
#  define imuinfo  _info
#  define imuerr   _err
#else
#  define imuinfo(...)
#  define imuerr   _err
#endif

#endif /* __APPS_EXAMPLES_IMU_SYSTEM_UTILS_CONFIG_H */
