/****************************************************************************
 * apps/examples/uav_states/drivers/sensors/icm42688p/icm42688p_driver.h
 *
 * ICM-42688-P 6-DOF IMU Driver
 * 3-axis Gyroscope + 3-axis Accelerometer
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_ICM42688P_DRIVER_H
#define __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_ICM42688P_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#include "../../drivers/comm/spi/spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Return codes */

#define ICM42688P_OK               0
#define ICM42688P_ERR_INIT        -1
#define ICM42688P_ERR_COMM        -2
#define ICM42688P_ERR_INVALID     -3
#define ICM42688P_ERR_TIMEOUT     -4
#define ICM42688P_ERR_SELFTEST    -5

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Tập dữ liệu cho một phép đo */
typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} data_t;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} icm42688p_raw_data_t;

/* Dữ liệu 3 trục (scaled - đã chuyển đổi) */

typedef struct
{
  float x;
  float y;
  float z;
} icm42688p_scaled_data_t;

/* Dữ liệu hoàn chỉnh từ IMU */

typedef struct
{
  icm42688p_scaled_data_t accel;      /* Gia tốc (g) */
  icm42688p_scaled_data_t gyro;       /* Tốc độ góc (dps) */
  float temperature;                   /* Nhiệt độ (°C) */
  uint64_t timestamp;                  /* Thời gian (microseconds) */
} icm42688p_data_t;

/* Cấu trúc quản lý thiết bị ICM42688P */

typedef struct
{
  spi_dev_t spi;                       /* SPI device handle */
  uint8_t current_bank;                /* Bank hiện tại */
  
  /* Calibration - Hiệu chuẩn gyroscope */
  icm42688p_scaled_data_t gyro_offset;
  
  /* Cấu hình Full Scale Range */
  uint8_t gyro_fs;                     /* Gyro FS selection */
  uint8_t accel_fs;                    /* Accel FS selection */
  
  /* Sensitivity (độ nhạy) */
  float gyro_sensitivity;              /* LSB/dps */
  float accel_sensitivity;             /* LSB/g */
  
  /* Thống kê */
  uint32_t sample_count;               /* Số mẫu đã đọc */
  uint32_t error_count;                /* Số lỗi */
  
  bool initialized;                    /* Trạng thái khởi tạo */
} icm42688p_dev_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif



#ifdef __cplusplus
}
#endif

#endif /* __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_ICM42688P_DRIVER_H */