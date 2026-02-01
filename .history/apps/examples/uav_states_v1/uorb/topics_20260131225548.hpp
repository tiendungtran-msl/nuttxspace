/****************************************************************************
 * apps/examples/uav_states_v1/uorb/topics.hpp
 *
 * Định nghĩa message cho các uORB Topic
 *
 * MỤC ĐÍCH:
 * - Định nghĩa cấu trúc dữ liệu trao đổi giữa các module.
 * - Mỗi struct là một "kiểu message" trong pub/sub.
 * - Giữ dạng POD để copy nhanh và an toàn.
 *
 * QUY ƯỚC ĐẶT TÊN (giống PX4):
 * - sensor_*    : Dữ liệu cảm biến thô
 * - vehicle_*   : Trạng thái xe đã xử lý
 * - estimator_* : Trạng thái nội bộ của estimator
 *
 * MỞ RỘNG TƯƠNG LAI:
 * - Thêm mag_sample_s cho magnetometer
 * - Thêm baro_sample_s cho barometer
 * - Thêm gps_sample_s cho GPS
 * - Thêm vehicle_local_position_s cho EKF2 output
 *
 ****************************************************************************/

#pragma once

#include <cstdint>

namespace uorb
{

/****************************************************************************
 * sensor_imu_s - Mẫu dữ liệu IMU
 *
 * Publisher: modules/sensors (imu_manager)
 * Subscriber: modules/estimator (attitude_estimator)
 *
 * Chứa dữ liệu accelerometer và gyroscope (đã bù bias)
 * từ ICM-42688-P hoặc IMU 6 trục tương tự.
 ****************************************************************************/

struct sensor_imu_s
{
    uint64_t timestamp_us;   // Sample timestamp (microseconds since boot)
    float accel[3];          // Acceleration [m/s²] (X, Y, Z in body frame)
    float gyro[3];           // Angular rate [rad/s] (X, Y, Z in body frame)
    float temperature;       // Die temperature [°C]
    uint8_t instance;        // Sensor instance (0-3 for multi-IMU)
    uint8_t _padding[3];     // Alignment padding
};

/****************************************************************************
 * vehicle_attitude_s - Tư thế (orientation) của phương tiện
 *
 * Publisher: modules/estimator
 * Subscriber: main (hiển thị), bộ điều khiển sau này
 *
 * Biểu diễn tư thế bằng quaternion.
 * Có thêm Euler angles để tiện hiển thị (tính từ quaternion).
 ****************************************************************************/

struct vehicle_attitude_s
{
    uint64_t timestamp_us;   // Timestamp when attitude was computed
    float q[4];              // Quaternion [w, x, y, z] (NED frame)
    float roll;              // Roll angle [rad] (-π to π)
    float pitch;             // Pitch angle [rad] (-π/2 to π/2)
    float yaw;               // Yaw angle [rad] (-π to π)
    float rollspeed;         // Angular velocity around X [rad/s]
    float pitchspeed;        // Angular velocity around Y [rad/s]
    float yawspeed;          // Angular velocity around Z [rad/s]
    uint8_t instance;        // Estimator instance (matches IMU instance)
    uint8_t _padding[7];     // Alignment padding
};

/****************************************************************************
 * estimator_status_s - Tình trạng estimator & chẩn đoán
 *
 * Publisher: modules/estimator
 * Subscriber: main (theo dõi sức khỏe), logger
 *
 * Hữu ích cho debug và ghi log bay.
 ****************************************************************************/

struct estimator_status_s
{
    uint64_t timestamp_us;
    float gyro_bias[3];      // Estimated gyro bias [rad/s]
    float accel_bias[3];     // Estimated accel bias [m/s²]
    float dt;                // Last update interval [s]
    uint32_t update_count;   // Number of estimator updates
    uint8_t instance;
    bool attitude_valid;     // true if attitude is trustworthy
    uint8_t _padding[2];
};

/****************************************************************************
 * sensor_calibration_s - Tham số hiệu chuẩn
 *
 * Dùng nội bộ để hiệu chỉnh cảm biến.
 * Có thể lưu/đọc từ flash.
 ****************************************************************************/

struct sensor_calibration_s
{
    float gyro_bias[3];      // Gyro bias [rad/s]
    float accel_bias[3];     // Accel bias [m/s²]
    float accel_scale[3];    // Accel scale factors
    float rotation[9];       // Rotation matrix (row-major)
    uint32_t calibration_id; // Unique ID for calibration
    bool valid;              // true if calibration is valid
    uint8_t _padding[3];
};

} // namespace uorb
