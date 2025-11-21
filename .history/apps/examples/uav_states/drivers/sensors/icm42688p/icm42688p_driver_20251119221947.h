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

/* Cấu trúc quản lý thiết bị ICM42688P */
typedef struct
{
  struct spi_dev_s dev;    /* Wrapper thiết bị của một ICM */
  uint

  data_t accel;           /* Gia tốc (g) */
  data_t gyro;            /* Tốc độ góc (dps) */
  float temperature;      /* Nhiệt độ (°C) */
  uint64_t timestamp;     /* Thời gian (microseconds) */

  /* Bank hiện tại */
  uint8_t current_bank;
  
  /* Calibration - Hiệu chuẩn gyroscope */
  data_t gyro_offset;
  
  /* Cấu hình Full Scale Range */
  uint8_t gyro_fs;                     /* Gyro FS selection */
  uint8_t accel_fs;                    /* Accel FS selection */
  
  /* Sensitivity (độ nhạy) */
  float gyro_sensitivity;              /* LSB/dps */
  float accel_sensitivity;             /* LSB/g */
  
  uint32_t sample_count;               /* Số mẫu đã đọc */
  
  bool initialized;                    /* Trạng thái khởi tạo */
} icm42688p_dev_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Cấu hình cho ICM được chọn */
int icm42688p_configure(icm42688p_dev_t *dev);

/* Ghi dữ liệu vào thanh ghi */
int icm42688p_write_register(icm42688p_dev_t *dev, uint8_t reg, uint8_t value);

/* Đọc dữ liệu từ thanh ghi */
int icm42688p_read_register(icm42688p_dev_t *dev, uint8_t reg, uint8_t *value);

/* Đọc các dữ liệu thu về từ cảm biến ICM */
int icm42688p_read_data(icm42688p_dev_t *dev);

/* Tính toán góc pitch, roll theo dữ liệu cảm biến */
int icm42688p_orientation(icm42688p_dev_t *dev, float *pitch, float *roll);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_ICM42688P_DRIVER_H */