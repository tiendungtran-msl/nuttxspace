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

/****************************************************************************
 * sensor_gps_s - Mẫu dữ liệu GPS
 *
 * Publisher: GPS driver (tương lai)
 * Subscriber: EKF2 module
 *
 * Chứa vị trí, vận tốc và độ chính xác từ GPS.
 ****************************************************************************/

struct sensor_gps_s
{
    uint64_t timestamp_us;   // Sample timestamp (microseconds since boot)
    double lat;              // Latitude [deg]
    double lon;              // Longitude [deg]
    float alt;               // Altitude MSL [m]
    float vel_n;             // Velocity North [m/s]
    float vel_e;             // Velocity East [m/s]
    float vel_d;             // Velocity Down [m/s]
    float hacc;              // Horizontal accuracy [m]
    float vacc;              // Vertical accuracy [m]
    float sacc;              // Speed accuracy [m/s]
    uint8_t fix_type;        // Fix type: 0=no fix, 2=2D, 3=3D, 4=RTK
    uint8_t nsats;           // Number of satellites
    uint8_t _padding[6];
};

/****************************************************************************
 * sensor_mag_s - Mẫu dữ liệu magnetometer
 *
 * Publisher: Mag driver (BMM150, ...)
 * Subscriber: EKF2 module
 *
 * Chứa từ trường đo được theo 3 trục.
 ****************************************************************************/

struct sensor_mag_s
{
    uint64_t timestamp_us;   // Sample timestamp
    float field[3];          // Magnetic field [Gauss] (X, Y, Z in body frame)
    float temperature;       // Sensor temperature [°C]
    uint8_t instance;        // Sensor instance
    uint8_t _padding[7];
};

/****************************************************************************
 * sensor_baro_s - Mẫu dữ liệu barometer
 *
 * Publisher: Baro driver (MS5611, ...)
 * Subscriber: EKF2 module
 *
 * Chứa áp suất và độ cao tính được.
 ****************************************************************************/

struct sensor_baro_s
{
    uint64_t timestamp_us;   // Sample timestamp
    float pressure;          // Pressure [Pa]
    float altitude;          // Altitude from pressure [m]
    float temperature;       // Sensor temperature [°C]
    uint8_t instance;        // Sensor instance
    uint8_t _padding[7];
};

/****************************************************************************
 * vehicle_local_position_s - Vị trí cục bộ NED
 *
 * Publisher: EKF2 module
 * Subscriber: Bộ điều khiển, navigation
 *
 * Vị trí và vận tốc trong hệ tọa độ NED (North-East-Down).
 * Gốc tọa độ là điểm khởi động hoặc home position.
 ****************************************************************************/

struct vehicle_local_position_s
{
    uint64_t timestamp_us;   // Timestamp

    // Vị trí NED
    float x;                 // North position [m]
    float y;                 // East position [m]
    float z;                 // Down position [m] (positive = dưới mặt đất)

    // Vận tốc NED
    float vx;                // North velocity [m/s]
    float vy;                // East velocity [m/s]
    float vz;                // Down velocity [m/s]

    // Gia tốc NED (dùng cho smoothing)
    float ax;                // North acceleration [m/s²]
    float ay;                // East acceleration [m/s²]
    float az;                // Down acceleration [m/s²]

    // GPS reference (origin)
    double ref_lat;          // Reference latitude [deg]
    double ref_lon;          // Reference longitude [deg]
    float ref_alt;           // Reference altitude [m MSL]

    // Validity flags
    bool xy_valid;           // True if xy position is valid
    bool z_valid;            // True if z position is valid
    bool v_xy_valid;         // True if xy velocity is valid
    bool v_z_valid;          // True if z velocity is valid

    uint8_t _padding[4];
};

/****************************************************************************
 * vehicle_global_position_s - Vị trí toàn cầu (GPS)
 *
 * Publisher: EKF2 module
 * Subscriber: GCS, navigation
 *
 * Vị trí dạng lat/lon/alt (geodetic).
 ****************************************************************************/

struct vehicle_global_position_s
{
    uint64_t timestamp_us;   // Timestamp
    double lat;              // Latitude [deg]
    double lon;              // Longitude [deg]
    float alt;               // Altitude MSL [m]
    float alt_ellipsoid;     // Altitude above WGS84 [m]
    float terrain_alt;       // Terrain altitude if available [m]
    bool terrain_alt_valid;
    uint8_t _padding[3];
};

/****************************************************************************
 * ekf2_status_s - Trạng thái EKF2
 *
 * Publisher: EKF2 module
 * Subscriber: Main (monitoring), logger
 *
 * Thông tin chi tiết về tình trạng EKF2 để debug/tuning.
 ****************************************************************************/

struct ekf2_status_s
{
    uint64_t timestamp_us;

    // Bias estimates
    float gyro_bias[3];      // Gyro bias [rad/s]
    float accel_bias[3];     // Accel bias [m/s²]

    // Innovation test ratios (chi-square)
    float gps_hpos_test_ratio;
    float gps_vpos_test_ratio;
    float gps_vel_test_ratio;
    float baro_hgt_test_ratio;
    float mag_heading_test_ratio;

    // Control status flags
    bool tilt_align;         // Tilt (roll/pitch) aligned
    bool yaw_align;          // Yaw aligned
    bool gps_fused;          // GPS được fusion
    bool baro_fused;         // Baro được fusion
    bool mag_fused;          // Mag heading được fusion

    uint8_t _padding[3];
};

} // namespace uorb
