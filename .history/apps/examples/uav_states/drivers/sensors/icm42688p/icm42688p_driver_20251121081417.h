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
  struct spi_dev_s *dev;  /* SPI device handle */
  uint8_t id;             /* ID của cảm biến (0-3) */
  bool initialized;       /* Trạng thái khởi tạo */

  data_t accel;           /* Gia tốc (g) */
  data_t gyro;            /* Tốc độ góc (dps) */
  data_t gyro_calib;
  float temp;             /* Nhiệt độ (°C) */
  uint64_t timestamp;     /* Thời gian (microseconds) */

  float pitch;            /* Góc pitch (°) */
  float roll;             /* Góc roll (°) */

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

} icm42688p_dev_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Khai báo cho ICM được chọn */
int icm42688p_init(icm42688p_dev_t *dev, uint8_t id);

/* Ghi dữ liệu vào thanh ghi */
int icm42688p_write_register(icm42688p_dev_t *dev, uint8_t reg, uint8_t value);

/* Đọc dữ liệu từ thanh ghi */
int icm42688p_read_register(icm42688p_dev_t *dev, uint8_t reg, uint8_t *value);

/* Đọc nhiều thanh ghi liên tiếp */
int icm42688p_read_registers(icm42688p_dev_t *dev, uint8_t reg, uint8_t *value, size_t length);

/* Đọc các dữ liệu thu về từ cảm biến ICM */
int icm42688p_get_data(icm42688p_dev_t *dev);

/* Hiệu chỉnh gyroscope */
int icm42688p_calibrate_gyro(icm42688p_dev_t *dev, int num_samples)

/* Chạy bài kiểm tra tự chẩn đoán */
int icm42688p_self_test(icm42688p_dev_t *dev)

/* In trạng thái hiện tại của cảm biến */
void icm42688p_print_status(icm42688p_dev_t *dev);

/* Tính toán góc pitch, roll theo dữ liệu cảm biến */
int icm42688p_orientation(icm42688p_dev_t *dev, float *pitch, float *roll);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_ICM42688P_DRIVER_H */