/****************************************************************************
 * apps/examples/imu_system/utils/config.h
 *
 * System Configuration for IMU System
 *
 * Hardware:
 *   - STM32H743 MCU @ 400 MHz
 *   - 4x ICM42688P IMU (SPI1: PA5, PA6, PA7)
 *     CS: PA4, PD11, PD12, PD13
 *   - 1x BMM150 Magnetometer (I2C1: PB6, PB7)
 *     Address: 0x10
 *   - Future: GPS on UART4 (PA0, PA1)
 *
 * Author: tiendungtran-msl
 * Date: 2025-11-14
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

#define IMU_NUM_ICM42688P        4      /* 4 IMU sensors on SPI */
#define IMU_NUM_MAGNETOMETERS    1      /* 1 magnetometer on I2C */

/* Communication Bus Configuration */

#define IMU_SPI_BUS              1      /* SPI1 for ICM42688P sensors */
#define IMU_I2C_BUS              1      /* I2C1 for BMM150 magnetometer */

/* SPI Device IDs (CS pins mapping) */

#define IMU_ICM0_DEVID           0      /* ICM42688P #0 - CS: PA4 */
#define IMU_ICM1_DEVID           1      /* ICM42688P #1 - CS: PD11 */
#define IMU_ICM2_DEVID           2      /* ICM42688P #2 - CS: PD12 */
#define IMU_ICM3_DEVID           3      /* ICM42688P #3 - CS: PD13 */

/* I2C Configuration for BMM150 */

#define IMU_BMM150_I2C_ADDR      0x10   /* 7-bit I2C address */
                                        /* Possible addresses: */
                                        /* 0x10 (SDO=GND) */
                                        /* 0x11 (SDO=VDDIO) */
                                        /* 0x12 (SDO=SCL) */
                                        /* 0x13 (SDO=SDA) */

/* Pin Definitions (Documentation) */

/* SPI1 Pins:
 *   PA5  - SPI1_SCK   (Clock)
 *   PA6  - SPI1_MISO  (Master In Slave Out)
 *   PA7  - SPI1_MOSI  (Master Out Slave In)
 *   PA4  - ICM0_CS    (Chip Select #0)
 *   PD11 - ICM1_CS    (Chip Select #1)
 *   PD12 - ICM2_CS    (Chip Select #2)
 *   PD13 - ICM3_CS    (Chip Select #3)
 */

/* I2C1 Pins:
 *   PB6  - I2C1_SCL   (Clock)
 *   PB7  - I2C1_SDA   (Data)
 */

/* Future GPS UART4:
 *   PA0  - UART4_TX
 *   PA1  - UART4_RX
 */

/* ========================================================================
 * Sensor Configuration
 * ======================================================================== */

/* ICM42688P Configuration */

#define IMU_ICM_ACCEL_RANGE      16     /* ±16g */
#define IMU_ICM_GYRO_RANGE       2000   /* ±2000 dps (degrees per second) */
#define IMU_ICM_ODR              1000   /* 1000 Hz output data rate */

/* BMM150 Configuration */

#define IMU_BMM_ODR              25     /* 25 Hz output data rate */
#define IMU_BMM_XY_REPS          9      /* Regular preset (XY axis) */
#define IMU_BMM_Z_REPS           15     /* Regular preset (Z axis) */

/* ========================================================================
 * Task Configuration
 * ======================================================================== */

/* Task Rates (Hz) */

#define IMU_SENSOR_RATE_HZ       1000   /* Sensor reading rate (1 kHz) */
#define IMU_FUSION_RATE_HZ       100    /* Sensor fusion update rate (100 Hz) */
#define IMU_LED_RATE_HZ          10     /* LED update rate (10 Hz) */

/* Task Priorities (higher number = higher priority) 
 * NuttX priority range: 1-255
 * Higher values = higher priority
 */

#define IMU_SENSOR_TASK_PRIORITY  150   /* Highest - real-time sensor reading */
#define IMU_FUSION_TASK_PRIORITY  140   /* High - sensor fusion computation */
#define IMU_LED_TASK_PRIORITY     100   /* Low - status indication */

/* Task Stack Sizes (bytes) */

#define IMU_SENSOR_TASK_STACKSIZE 4096  /* 4 KB - Sensor task */
#define IMU_FUSION_TASK_STACKSIZE 4096  /* 4 KB - Fusion task */
#define IMU_LED_TASK_STACKSIZE    1024  /* 1 KB - LED task */

/* Task Period Calculations (microseconds) */

#define IMU_SENSOR_PERIOD_US     (1000000 / IMU_SENSOR_RATE_HZ)
#define IMU_FUSION_PERIOD_US     (1000000 / IMU_FUSION_RATE_HZ)
#define IMU_LED_PERIOD_US        (1000000 / IMU_LED_RATE_HZ)

/* ========================================================================
 * Data Queue Configuration
 * ======================================================================== */

/* Queue Sizes (number of elements) */

#define IMU_SENSOR_QUEUE_SIZE    64     /* Sensor data queue (64 packets) */
#define IMU_FUSION_QUEUE_SIZE    32     /* Fusion result queue (32 results) */

/* Queue Memory Usage:
 * Sensor Queue: 64 × ~1.5KB = ~96 KB
 * Fusion Queue: 32 × 64B = ~2 KB
 * Total: ~98 KB
 */

/* ========================================================================
 * Algorithm Configuration
 * ======================================================================== */

/* Madgwick AHRS Filter */

#define IMU_MADGWICK_BETA        0.1f   /* Filter gain (0.0 - 1.0) */
                                        /* Higher = faster convergence */
                                        /* Lower = smoother output */

#define IMU_MADGWICK_SAMPLE_FREQ ((float)IMU_FUSION_RATE_HZ)

/* Complementary Filter (alternative) */

#define IMU_COMP_FILTER_ALPHA    0.98f  /* Gyro weight (0.0 - 1.0) */
                                        /* 0.98 = 98% gyro, 2% accel/mag */

/* ========================================================================
 * Calibration Configuration
 * ======================================================================== */

/* Gyroscope Calibration */

#define IMU_GYRO_CALIB_SAMPLES   1000   /* Number of samples for gyro cal */
#define IMU_GYRO_CALIB_TIME_MS   (IMU_GYRO_CALIB_SAMPLES * \
                                  1000 / IMU_ICM_ODR)

/* Magnetometer Calibration */

#define IMU_MAG_CALIB_SAMPLES    500    /* Number of samples for mag cal */
#define IMU_MAG_CALIB_TIME_S     (IMU_MAG_CALIB_SAMPLES / IMU_BMM_ODR)

/* Accelerometer Calibration (future) */

#define IMU_ACCEL_CALIB_SAMPLES  1000   /* Number of samples for accel cal */

/* ========================================================================
 * LED Configuration
 * ======================================================================== */

/* LED Blink Patterns (milliseconds) */

#define IMU_LED_FAST_BLINK_MS    100    /* Fast blink - error */
#define IMU_LED_SLOW_BLINK_MS    500    /* Slow blink - calibration */
#define IMU_LED_HEARTBEAT_MS     1000   /* Heartbeat - normal operation */

/* ========================================================================
 * Error Handling Configuration
 * ======================================================================== */

/* Error Thresholds */

#define IMU_MAX_QUEUE_OVERFLOW   100    /* Max queue overflows before warning */
#define IMU_MAX_SENSOR_ERRORS    50     /* Max sensor read errors before alert */
#define IMU_SENSOR_TIMEOUT_MS    1000   /* Sensor read timeout */

/* Watchdog */

#define IMU_ENABLE_WATCHDOG      0      /* Enable task watchdog (0=off, 1=on) */
#define IMU_WATCHDOG_TIMEOUT_MS  5000   /* Watchdog timeout */

/* ========================================================================
 * Debug Configuration
 * ======================================================================== */

/* Debug Output Levels */

#define IMU_DEBUG_NONE           0      /* No debug output */
#define IMU_DEBUG_ERROR          1      /* Errors only */
#define IMU_DEBUG_WARN           2      /* Warnings and errors */
#define IMU_DEBUG_INFO           3      /* Info, warnings, errors */
#define IMU_DEBUG_VERBOSE        4      /* All debug output */

#ifndef IMU_DEBUG_LEVEL
#  define IMU_DEBUG_LEVEL        IMU_DEBUG_INFO
#endif

/* Performance Monitoring */

#define IMU_ENABLE_PERF_STATS    1      /* Enable performance statistics */
#define IMU_PERF_REPORT_INTERVAL 10     /* Report interval (seconds) */

/* ========================================================================
 * Safety Limits
 * ======================================================================== */

/* Sensor Value Limits (sanity checks) */

#define IMU_MAX_ACCEL_G          20.0f  /* Maximum acceleration (g) */
#define IMU_MAX_GYRO_DPS         2500.0f /* Maximum rotation rate (dps) */
#define IMU_MAX_MAG_UT           200.0f /* Maximum magnetic field (uT) */

/* Temperature Limits */

#define IMU_MIN_TEMP_C           -40.0f /* Minimum operating temp */
#define IMU_MAX_TEMP_C           85.0f  /* Maximum operating temp */

/* ========================================================================
 * Memory Pool Configuration
 * ======================================================================== */

/* Static Memory Allocation */

#define IMU_USE_STATIC_ALLOC     1      /* Use static allocation (0=dynamic) */

/* DMA Configuration (if enabled) */

#define IMU_USE_DMA              0      /* Use DMA for SPI/I2C (0=off, 1=on) */
#define IMU_DMA_BUFFER_SIZE      512    /* DMA buffer size (bytes) */

/* ========================================================================
 * Feature Flags
 * ======================================================================== */

/* Enable/Disable Features */

#define IMU_ENABLE_ACCEL         1      /* Enable accelerometer */
#define IMU_ENABLE_GYRO          1      /* Enable gyroscope */
#define IMU_ENABLE_MAG           1      /* Enable magnetometer */
#define IMU_ENABLE_TEMP          1      /* Enable temperature sensor */

#define IMU_ENABLE_CALIBRATION   1      /* Enable calibration functions */
#define IMU_ENABLE_SELF_TEST     1      /* Enable self-test functions */
#define IMU_ENABLE_FUSION        1      /* Enable sensor fusion */

/* Future Features */

#define IMU_ENABLE_GPS           0      /* Enable GPS (not implemented) */
#define IMU_ENABLE_BARO          0      /* Enable barometer (future) */
#define IMU_ENABLE_LOGGING       0      /* Enable data logging (future) */
#define IMU_ENABLE_TELEMETRY     0      /* Enable telemetry (future) */

/* ========================================================================
 * Type Definitions
 * ======================================================================== */

/* System State */

typedef struct
{
  bool running;                         /* System running flag */
  bool calibration_mode;                /* Calibration mode flag */
  pthread_mutex_t mutex;                /* System state mutex */
  
  /* Task handles */
  pthread_t sensor_thread;              /* Sensor task handle */
  pthread_t fusion_thread;              /* Fusion task handle */
  pthread_t led_thread;                 /* LED task handle */
  
  /* Statistics */
  uint32_t sensor_cycles;               /* Sensor task cycles */
  uint32_t fusion_cycles;               /* Fusion task cycles */
  uint32_t queue_overflows;             /* Queue overflow count */
  uint32_t sensor_errors;               /* Sensor error count */
} system_state_t;

/* ========================================================================
 * Global Variables (External)
 * ======================================================================== */

#ifdef __cplusplus
extern "C"
{
#endif

/* System state (defined in main) */

extern system_state_t g_system_state;

/* Data queues (defined in main) */

extern struct data_queue_s g_sensor_queue;
extern struct data_queue_s g_fusion_queue;

#ifdef __cplusplus
}
#endif

/* ========================================================================
 * Compile-Time Checks
 * ======================================================================== */

/* Verify configuration sanity */

#if IMU_SENSOR_RATE_HZ > 1000
#  error "IMU_SENSOR_RATE_HZ too high (max 1000 Hz)"
#endif

#if IMU_FUSION_RATE_HZ > IMU_SENSOR_RATE_HZ
#  error "IMU_FUSION_RATE_HZ cannot exceed IMU_SENSOR_RATE_HZ"
#endif

#if IMU_SENSOR_QUEUE_SIZE < 16
#  warning "IMU_SENSOR_QUEUE_SIZE is very small, may cause overflows"
#endif

#if IMU_MADGWICK_BETA < 0.0f || IMU_MADGWICK_BETA > 1.0f
#  error "IMU_MADGWICK_BETA must be between 0.0 and 1.0"
#endif

#if IMU_NUM_ICM42688P != 4
#  error "This configuration requires exactly 4 ICM42688P sensors"
#endif

/* ========================================================================
 * Macros
 * ======================================================================== */

/* Conversion macros */

#define DEG_TO_RAD(deg)          ((deg) * 0.017453292519943295f)
#define RAD_TO_DEG(rad)          ((rad) * 57.29577951308232f)
#define G_TO_MS2(g)              ((g) * 9.80665f)
#define MS2_TO_G(ms2)            ((ms2) / 9.80665f)

/* Utility macros */

#define MIN(a, b)                (((a) < (b)) ? (a) : (b))
#define MAX(a, b)                (((a) > (b)) ? (a) : (b))
#define CLAMP(x, min, max)       (MIN(MAX((x), (min)), (max)))
#define ABS(x)                   (((x) < 0) ? -(x) : (x))

/* Array size */

#define ARRAY_SIZE(arr)          (sizeof(arr) / sizeof((arr)[0]))

/* ========================================================================
 * Version Information
 * ======================================================================== */

#define IMU_SYSTEM_VERSION_MAJOR 1
#define IMU_SYSTEM_VERSION_MINOR 0
#define IMU_SYSTEM_VERSION_PATCH 0

#define IMU_SYSTEM_VERSION_STRING "1.0.0"
#define IMU_SYSTEM_BUILD_DATE     __DATE__
#define IMU_SYSTEM_BUILD_TIME     __TIME__

#endif /* __APPS_EXAMPLES_IMU_SYSTEM_UTILS_CONFIG_H */