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

/* Dữ liệu 3 trục (raw - chưa chuyển đổi) */

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

/****************************************************************************
 * Name: icm42688p_init
 *
 * Description:
 *   Khởi tạo ICM42688P IMU
 *
 * Input Parameters:
 *   dev       - Pointer đến cấu trúc device
 *   bus       - SPI bus number
 *
 * Returned Value:
 *   ICM42688P_OK on success; Negative error code on failure
 *
 ****************************************************************************/

int icm42688p_init(icm42688p_dev_t *dev, uint8_t bus);

/****************************************************************************
 * Name: icm42688p_deinit
 *
 * Description:
 *   Dừng và đóng ICM42688P
 *
 ****************************************************************************/

int icm42688p_deinit(icm42688p_dev_t *dev);

/****************************************************************************
 * Name: icm42688p_reset
 *
 * Description:
 *   Soft reset ICM42688P
 *
 ****************************************************************************/

int icm42688p_reset(icm42688p_dev_t *dev);

/****************************************************************************
 * Name: icm42688p_read_data
 *
 * Description:
 *   Đọc dữ liệu Accel + Gyro + Temperature
 *
 ****************************************************************************/

int icm42688p_read_data(icm42688p_dev_t *dev, icm42688p_data_t *data);

/****************************************************************************
 * Name: icm42688p_calibrate_gyro
 *
 * Description:
 *   Hiệu chuẩn Gyroscope (tính toán offset)
 *   Thiết bị phải đứng yên trong quá trình hiệu chuẩn!
 *
 * Input Parameters:
 *   dev         - Pointer đến device structure
 *   num_samples - Số mẫu để lấy trung bình (khuyến nghị: 200-500)
 *
 ****************************************************************************/

int icm42688p_calibrate_gyro(icm42688p_dev_t *dev, int num_samples);

/****************************************************************************
 * Name: icm42688p_self_test
 *
 * Description:
 *   Thực hiện self-test để kiểm tra sensor hoạt động
 *
 ****************************************************************************/

int icm42688p_self_test(icm42688p_dev_t *dev);

/****************************************************************************
 * Name: icm42688p_set_gyro_range
 *
 * Description:
 *   Đặt Full Scale Range cho Gyroscope
 *
 * Input Parameters:
 *   dev   - Pointer đến device structure
 *   range - 0=±250dps, 1=±500dps, 2=±1000dps, 3=±2000dps
 *
 ****************************************************************************/

int icm42688p_set_gyro_range(icm42688p_dev_t *dev, uint8_t range);

/****************************************************************************
 * Name: icm42688p_set_accel_range
 *
 * Description:
 *   Đặt Full Scale Range cho Accelerometer
 *
 * Input Parameters:
 *   dev   - Pointer đến device structure
 *   range - 0=±2g, 1=±4g, 2=±8g, 3=±16g
 *
 ****************************************************************************/

int icm42688p_set_accel_range(icm42688p_dev_t *dev, uint8_t range);

/****************************************************************************
 * Name: icm42688p_set_odr
 *
 * Description:
 *   Đặt Output Data Rate cho cả Accel và Gyro
 *
 * Input Parameters:
 *   dev     - Pointer đến device structure
 *   odr_val - ODR value (xem ICM42688P_GYRO_CONFIG0_ODR_xxx)
 *
 ****************************************************************************/

int icm42688p_set_odr(icm42688p_dev_t *dev, uint8_t odr_val);

/****************************************************************************
 * Name: icm42688p_print_status
 *
 * Description:
 *   In ra trạng thái của sensor
 *
 ****************************************************************************/

void icm42688p_print_status(icm42688p_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_ICM42688P_DRIVER_H */