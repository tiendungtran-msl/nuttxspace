/****************************************************************************
 * apps/examples/uav_states/drivers/sensors/icm42688p/icm42688p_driver.h
 *
 * ICM-42688-P 6-DOF IMU Driver
 * 3-axis Gyroscope + 3-axis Accelerometer
 * Optimized version with enhanced error handling
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
#include <pthread.h>

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
#define ICM42688P_ERR_MUTEX       -6

/* Device state flags */
#define ICM42688P_STATE_UNINITIALIZED  0x00
#define ICM42688P_STATE_INITIALIZED    0x01
#define ICM42688P_STATE_CALIBRATED     0x02
#define ICM42688P_STATE_ERROR          0x80

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Tập dữ liệu cho một phép đo */
typedef struct
{
  float x;
  float y;
  float z;
} data_t;

/* Cấu trúc quản lý thiết bị ICM42688P */
typedef struct
{
  /* Device identification */
  uint8_t id;                          /* ID của cảm biến (0-3) */
  uint8_t state;                       /* Device state flags */
  
  /* Thread safety */
  pthread_mutex_t lock;                /* Mutex cho thread safety */
  bool lock_initialized;               /* Mutex init status */
  
  /* Current sensor data */
  data_t accel;                        /* Gia tốc (g) */
  data_t gyro;                         /* Tốc độ góc (dps) - raw */
  data_t gyro_calib;                   /* Tốc độ góc (dps) - calibrated */
  float temp;                          /* Nhiệt độ (°C) */
  uint64_t timestamp;                  /* Thời gian (microseconds) */

  /* Orientation */
  float pitch;                         /* Góc pitch (°) */
  float roll;                          /* Góc roll (°) */

  /* Internal state */
  uint8_t current_bank;                /* Bank hiện tại */
  
  /* Calibration data */
  data_t gyro_offset;                  /* Gyro offset (dps) */
  bool is_calibrated;                  /* Calibration status */
  
  /* Configuration */
  uint8_t gyro_fs;                     /* Gyro FS selection */
  uint8_t accel_fs;                    /* Accel FS selection */ 
  float gyro_sensitivity;              /* LSB/dps */
  float accel_sensitivity;             /* LSB/g */
  
  /* Statistics */
  uint32_t sample_count;               /* Số mẫu đã đọc */
  uint32_t error_count;                /* Số lỗi đã gặp */
  uint32_t last_error_code;            /* Mã lỗi cuối cùng */

} icm42688p_dev_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Khởi tạo cảm biến icm với thứ tự id */
int icm42688p_init(icm42688p_dev_t *dev, uint8_t id);

/****************************************************************************
 * Name: icm42688p_deinit
 * 
 * Description:
 *   Deinitialize sensor và cleanup resources
 ****************************************************************************/
int icm42688p_deinit(icm42688p_dev_t *dev);

/****************************************************************************
 * Name: icm42688p_write_register
 * 
 * Description:
 *   Ghi dữ liệu vào thanh ghi với retry mechanism
 ****************************************************************************/
int icm42688p_write_register(icm42688p_dev_t *dev, uint8_t reg, uint8_t value);

/****************************************************************************
 * Name: icm42688p_read_register
 * 
 * Description:
 *   Đọc dữ liệu từ thanh ghi với retry mechanism
 ****************************************************************************/
int icm42688p_read_register(icm42688p_dev_t *dev, uint8_t reg, uint8_t *value);

/****************************************************************************
 * Name: icm42688p_read_registers
 * 
 * Description:
 *   Đọc nhiều thanh ghi liên tiếp với burst read
 ****************************************************************************/
int icm42688p_read_registers(icm42688p_dev_t *dev, uint8_t reg, 
                             uint8_t *buffer, size_t length);

/****************************************************************************
 * Name: icm42688p_get_data
 * 
 * Description:
 *   Đọc dữ liệu cảm biến với integrity check và mutex protection
 ****************************************************************************/
int icm42688p_get_data(icm42688p_dev_t *dev);

/****************************************************************************
 * Name: icm42688p_calibrate_gyro
 * 
 * Description:
 *   Hiệu chỉnh gyroscope với outlier rejection
 ****************************************************************************/
int icm42688p_calibrate_gyro(icm42688p_dev_t *dev, int num_samples);

/****************************************************************************
 * Name: icm42688p_self_test
 * 
 * Description:
 *   Chạy bài kiểm tra tự chẩn đoán
 ****************************************************************************/
int icm42688p_self_test(icm42688p_dev_t *dev);

/****************************************************************************
 * Name: icm42688p_print_status
 * 
 * Description:
 *   In trạng thái hiện tại của cảm biến
 ****************************************************************************/
void icm42688p_print_status(icm42688p_dev_t *dev);

/****************************************************************************
 * Name: icm42688p_orientation
 * 
 * Description:
 *   Tính toán góc pitch, roll từ dữ liệu accelerometer
 ****************************************************************************/
int icm42688p_orientation(icm42688p_dev_t *dev);

/****************************************************************************
 * Name: icm42688p_reset
 * 
 * Description:
 *   Soft reset sensor
 ****************************************************************************/
int icm42688p_reset(icm42688p_dev_t *dev);

/****************************************************************************
 * Name: icm42688p_lock
 * 
 * Description:
 *   Lock device mutex for thread-safe access
 ****************************************************************************/
static inline int icm42688p_lock(icm42688p_dev_t *dev)
{
  if (!dev || !dev->lock_initialized)
    {
      return ICM42688P_ERR_INVALID;
    }
  
  if (pthread_mutex_lock(&dev->lock) != 0)
    {
      return ICM42688P_ERR_MUTEX;
    }
  
  return ICM42688P_OK;
}

/****************************************************************************
 * Name: icm42688p_unlock
 * 
 * Description:
 *   Unlock device mutex
 ****************************************************************************/
static inline int icm42688p_unlock(icm42688p_dev_t *dev)
{
  if (!dev || !dev->lock_initialized)
    {
      return ICM42688P_ERR_INVALID;
    }
  
  if (pthread_mutex_unlock(&dev->lock) != 0)
    {
      return ICM42688P_ERR_MUTEX;
    }
  
  return ICM42688P_OK;
}

#ifdef __cplusplus
}
#endif

#endif /* __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_ICM42688P_DRIVER_H */