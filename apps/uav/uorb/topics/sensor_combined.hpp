/****************************************************************************
 * apps/uav/uorb/topics/sensor_combined.hpp
 *
 * SENSOR COMBINED - Dữ liệu cảm biến đã được fuse từ nhiều nguồn
 *
 * MỤC ĐÍCH:
 * - Cung cấp dữ liệu IMU đã được voting/weighted average
 * - Một topic duy nhất cho EKF consume
 * - Bao gồm thông tin về health và số IMU được sử dụng
 *
 ****************************************************************************/

#ifndef __UAV_UORB_TOPICS_SENSOR_COMBINED_HPP
#define __UAV_UORB_TOPICS_SENSOR_COMBINED_HPP

#include <uav/uorb/orb_defines.hpp>
#include <stdint.h>

/**
 * @brief Dữ liệu IMU đã được kết hợp từ nhiều sensor
 */
struct sensor_combined_s {
    uint64_t timestamp_us;          /**< Timestamp của fusion output */

    /* Fused IMU data */
    float gyro[3];                  /**< Fused angular rate (rad/s) */
    float accel[3];                 /**< Fused acceleration (m/s²) */
    float gyro_integral[3];         /**< Gyro integral over dt (rad) */
    float accel_integral[3];        /**< Accel integral over dt (m/s) */
    float dt;                       /**< Integration time (s) */

    /* Sensor health info */
    uint8_t num_imus_used;          /**< Số IMU được sử dụng trong fusion */
    uint8_t healthy_mask;           /**< Bitmask của IMU healthy */
    uint8_t fusion_mode;            /**< 0=VOTING, 1=WEIGHTED, 2=PRIMARY */

    /* Quality indicators */
    float gyro_noise;               /**< Estimated gyro noise (rad/s) */
    float accel_noise;              /**< Estimated accel noise (m/s²) */

    /* Validity */
    bool valid;                     /**< true nếu data valid */
};

ORB_DECLARE(sensor_combined);

#endif /* __UAV_UORB_TOPICS_SENSOR_COMBINED_HPP */
