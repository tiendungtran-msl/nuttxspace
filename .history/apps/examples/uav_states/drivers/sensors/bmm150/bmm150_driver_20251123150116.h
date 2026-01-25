/****************************************************************************
 * apps/examples/uav_states/drivers/sensors/bmm150/bmm150_driver.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_BMM150_DRIVER_H
#define __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_BMM150_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#include "../../drivers/comm/i2c/i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMM150_OK               0
#define BMM150_ERR_INIT        -1
#define BMM150_ERR_COMM        -2
#define BMM150_ERR_INVALID     -3
#define BMM150_ERR_TIMEOUT     -4
#define BMM150_ERR_OVERFLOW    -5

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Factory trim/compensation data structure */
typedef struct
{
  int8_t dig_x1;
  int8_t dig_y1;
  int8_t dig_x2;
  int8_t dig_y2;
  uint16_t dig_z1;
  int16_t dig_z2;
  int16_t dig_z3;
  int16_t dig_z4;
  uint8_t dig_xy1;
  int8_t dig_xy2;
  uint16_t dig_xyz1;
} bmm150_trim_data_t;

/* Magnetometer raw data */
typedef struct
{
  int16_t raw_x;
  int16_t raw_y;
  int16_t raw_z;
  uint16_t raw_rhall;
} bmm150_raw_data_t;

/* Magnetometer compensated data */
typedef struct
{
  float x;            /* µT (microtesla) */
  float y;            /* µT */
  float z;            /* µT */
  uint64_t timestamp; /* microseconds */
} bmm150_mag_data_t;

/* Calibration data */
typedef struct
{
  float offset[3];    /* Hard iron offset [x, y, z] */
  float scale[3];     /* Soft iron scale [x, y, z] */
  bool valid;
} bmm150_cal_data_t;

/* BMM150 device handle */
typedef struct
{
  i2c_master_s i2c;              /* I2C device handle */
  bmm150_trim_data_t trim;       /* Factory trim data */
  bmm150_cal_data_t cal;         /* User calibration data */
  bool initialized;
  uint32_t sample_count;
  uint32_t error_count;
} bmm150_dev_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Khởi tạo BMM150 theo đúng ánh xạ thiết bị I2C với địa chỉ addr */
int bmm150_init(bmm150_dev_t *dev, uint8_t addr);

/* Hủy khởi tạo */

int bmm150_deinit(bmm150_dev_t *dev);

/****************************************************************************
 * Name: bmm150_read_raw
 *
 * Description:
 *   Read raw magnetometer and hall resistance data
 *
 * Input Parameters:
 *   dev      - Pointer to bmm150_dev_t structure
 *   raw_data - Pointer to store raw data
 *
 * Returned Value:
 *   BMM150_OK on success; Negative error code on failure
 *
 ****************************************************************************/

int bmm150_read_raw(bmm150_dev_t *dev, bmm150_raw_data_t *raw_data);

/****************************************************************************
 * Name: bmm150_read_mag
 *
 * Description:
 *   Read and compensate magnetometer data
 *
 * Input Parameters:
 *   dev      - Pointer to bmm150_dev_t structure
 *   mag_data - Pointer to store compensated magnetic field data
 *
 * Returned Value:
 *   BMM150_OK on success; Negative error code on failure
 *
 ****************************************************************************/

int bmm150_read_mag(bmm150_dev_t *dev, bmm150_mag_data_t *mag_data);

/****************************************************************************
 * Name: bmm150_self_test
 *
 * Description:
 *   Perform BMM150 self-test
 *
 * Input Parameters:
 *   dev - Pointer to bmm150_dev_t structure
 *
 * Returned Value:
 *   BMM150_OK on success; Negative error code on failure
 *
 ****************************************************************************/

int bmm150_self_test(bmm150_dev_t *dev);

/****************************************************************************
 * Name: bmm150_calibrate
 *
 * Description:
 *   Perform magnetometer calibration (hard/soft iron)
 *
 * Input Parameters:
 *   dev         - Pointer to bmm150_dev_t structure
 *   num_samples - Number of samples to collect for calibration
 *
 * Returned Value:
 *   BMM150_OK on success; Negative error code on failure
 *
 ****************************************************************************/

int bmm150_calibrate(bmm150_dev_t *dev, int num_samples);

/****************************************************************************
 * Name: bmm150_set_preset_mode
 *
 * Description:
 *   Set BMM150 preset mode (Low Power, Regular, Enhanced, High Accuracy)
 *
 * Input Parameters:
 *   dev  - Pointer to bmm150_dev_t structure
 *   mode - Preset mode (0=Low Power, 1=Regular, 2=Enhanced, 3=High Accuracy)
 *
 * Returned Value:
 *   BMM150_OK on success; Negative error code on failure
 *
 ****************************************************************************/

int bmm150_set_preset_mode(bmm150_dev_t *dev, uint8_t mode);

/****************************************************************************
 * Name: bmm150_set_odr
 *
 * Description:
 *   Set output data rate
 *
 * Input Parameters:
 *   dev - Pointer to bmm150_dev_t structure
 *   odr - Output data rate (use BMM150_ODR_xxx defines)
 *
 * Returned Value:
 *   BMM150_OK on success; Negative error code on failure
 *
 ****************************************************************************/

int bmm150_set_odr(bmm150_dev_t *dev, uint8_t odr);

/****************************************************************************
 * Name: bmm150_get_heading
 *
 * Description:
 *   Calculate magnetic heading (compass direction)
 *
 * Input Parameters:
 *   dev     - Pointer to bmm150_dev_t structure
 *   heading - Pointer to store heading in degrees (0-360)
 *
 * Returned Value:
 *   BMM150_OK on success; Negative error code on failure
 *
 ****************************************************************************/

int bmm150_get_heading(bmm150_dev_t *dev, float *heading);

/****************************************************************************
 * Name: bmm150_print_status
 *
 * Description:
 *   Print BMM150 device status
 *
 * Input Parameters:
 *   dev - Pointer to bmm150_dev_t structure
 *
 ****************************************************************************/

void bmm150_print_status(bmm150_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_BMM150_DRIVER_H */