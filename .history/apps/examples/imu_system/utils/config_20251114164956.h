/****************************************************************************
 * apps/examples/imu_system/utils/config.h
 *
 * System Configuration for IMU System
 *
 * Hardware:
 *   - STM32H743 MCU
 *   - 4x ICM42688P IMU (SPI1: PA5, PA6, PA7)
 *     CS: PA4, PD11, PD12, PD13
 *   - 1x BMM150 Magnetometer (I2C1: PB6, PB7)
 *     Address: 0x10
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_IMU_SYSTEM_UTILS_CONFIG_H
#define __APPS_EXAMPLES_IMU_SYSTEM_UTILS_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ========================================================================
 * Hardware Configuration
 * ======================================================================== */

/* Number of sensors */

#define IMU_NUM_ICM42688P        4      /* 4 IMU sensors */
#define IMU_NUM_MAGNETOMETERS    1      /* 1 magnetometer */

/* Communication Bus Configuration */

#define IMU_SPI_BUS              1      /* SPI1 for ICM42688P sensors */
#define IMU_I2C_BUS              1      /* I2C1 for BMM150 magnetometer */

/* SPI Device IDs (CS pins) */

#define IMU_ICM0_DEVID           0      /* ICM42688P #0 - CS: PA4 */
#define IMU_ICM1_DEVID           1      /* ICM42688P #1 - CS: PD11 */
#define IMU_ICM2_DEVID           2      /* ICM42688P #2 - CS: PD12 */
#define IMU_ICM3_DEVID           3      /* ICM42688P #3 - CS: PD13 */

/* I2C Configuration for BMM150 */

#define IMU_BMM150_I2C_ADDR      0x10   /* 7-bit I2C address */
                                        /* Possible: 0x10, 0x11, 0x12, 0x13 */
                                        /* Check SDO pin connection */

/* Pin Definitions (for reference) */

/* SPI1 Pins:
 *   PA5  - SPI1_SCK
 *   PA6  - SPI1_MISO
 *   PA7  - SPI1_MOSI
 *   PA4  - ICM0_CS
 *   PD11 - ICM1_CS
 *   PD12 - ICM2_CS
 *   PD13 - ICM3_CS
 */

/* I2C1 Pins:
 *   PB6  - I2C1_SCL
 *   PB7  - I2C1_SDA
 */

/* ========================================================================
 * Sensor Configuration
 * ======================================================================== */

/* ICM42688P Configuration */

#define IMU_ICM_ACCEL_RANGE      16     /* ±16g */
#define IMU_ICM_GYRO_RANGE       2000   /* ±2000 dps */
#define IMU_ICM_ODR              1000   /* 1000 Hz output data rate */

/* BMM150 Configuration */

#define IMU_BMM_ODR              25     /* 25 Hz output data rate */
#define IMU_BMM_XY_REPS          9      /* Regular preset */
#define IMU_BMM_Z_REPS           15     /* Regular preset */

/* ========================================================================
 * Task Configuration
 * ======================================================================== */

/* Task Rates (Hz) */

#define IMU_SENSOR_RATE_HZ       1000   /* Sensor reading rate */
#define IMU_FUSION_RATE_HZ       100    /* Sensor fusion update rate */
#define IMU_LED_RATE_HZ          10     /* LED update rate */

/* Task Priorities (higher number = higher priority) */

#define IMU_SENSOR_TASK_PRIORITY  150   /* Highest - real-time sensor */
#define IMU_FUSION_TASK_PRIORITY  140   /* High - sensor fusion */
#define IMU_LED_TASK_PRIORITY     100   /* Low - status indication */

/* Task Stack Sizes (bytes) */

#define IMU_SENSOR_TASK_STACKSIZE 4096  /* Sensor task stack */
#define IMU_FUSION_TASK_STACKSIZE 4096  /* Fusion task stack */
#define IMU_LED_TASK_STACKSIZE    1024  /* LED task stack */

/* ========================================================================
 * Data Queue Configuration
 * ======================================================================== */

/* Queue Sizes (number of elements) */

#define IMU_SENSOR_QUEUE_SIZE    64     /* Sensor data queue */
#define IMU_