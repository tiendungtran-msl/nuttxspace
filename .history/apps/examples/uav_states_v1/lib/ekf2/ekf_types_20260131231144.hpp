/****************************************************************************
 * apps/examples/uav_states_v1/lib/ekf2/ekf_types.hpp
 *
 * Định nghĩa kiểu dữ liệu cho EKF2 - Lấy cảm hứng từ PX4
 *
 * MỤC ĐÍCH:
 * - Định nghĩa state vector (quaternion, velocity, position, biases).
 * - Định nghĩa các sample types cho từng loại cảm biến.
 * - Định nghĩa tham số bộ lọc.
 *
 * STATE VECTOR (16 states):
 * - [0-3]   Quaternion (q0, q1, q2, q3) - tư thế
 * - [4-6]   Velocity NED (vn, ve, vd) - vận tốc
 * - [7-9]   Position NED (pn, pe, pd) - vị trí
 * - [10-12] Gyro bias (bgx, bgy, bgz) - bias gyro
 * - [13-15] Accel bias (bax, bay, baz) - bias accel
 *
 * MỞ RỘNG:
 * - Có thể thêm mag_I, mag_B, wind, terrain như PX4 sau này.
 *
 ****************************************************************************/

#pragma once

#include <cstdint>
#include <cmath>
#include <cstring>

namespace ekf2
{

/****************************************************************************
 * Cấu hình kích thước state
 ****************************************************************************/

// Số phần tử state vector
static constexpr int STATE_SIZE = 16;

// Index các thành phần trong state vector
struct StateIdx
{
    static constexpr int QUAT = 0;      // Quaternion [4]
    static constexpr int VEL = 4;       // Velocity NED [3]
    static constexpr int POS = 7;       // Position NED [3]
    static constexpr int GYRO_BIAS = 10; // Gyro bias [3]
    static constexpr int ACCEL_BIAS = 13; // Accel bias [3]
};

/****************************************************************************
 * Vector3f - Vector 3D đơn giản
 ****************************************************************************/

struct Vector3f
{
    float x, y, z;

    Vector3f() : x(0), y(0), z(0) {}
    Vector3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vector3f(const float v[3]) : x(v[0]), y(v[1]), z(v[2]) {}

    float norm() const { return sqrtf(x*x + y*y + z*z); }
    float norm_sq() const { return x*x + y*y + z*z; }

    void normalize()
    {
        float n = norm();
        if (n > 1e-6f) {
            float inv = 1.0f / n;
            x *= inv; y *= inv; z *= inv;
        }
    }

    Vector3f normalized() const
    {
        Vector3f v = *this;
        v.normalize();
        return v;
    }

    void zero() { x = y = z = 0.0f; }

    // Toán tử
    Vector3f operator+(const Vector3f& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vector3f operator-(const Vector3f& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vector3f operator*(float s) const { return {x*s, y*s, z*s}; }
    Vector3f& operator+=(const Vector3f& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vector3f& operator-=(const Vector3f& o) { x-=o.x; y-=o.y; z-=o.z; return *this; }

    // Dot product
    float dot(const Vector3f& o) const { return x*o.x + y*o.y + z*o.z; }

    // Cross product
    Vector3f cross(const Vector3f& o) const
    {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }

    // Truy cập theo index
    float& operator[](int i) { return (&x)[i]; }
    float operator[](int i) const { return (&x)[i]; }
};

/****************************************************************************
 * Quatf - Quaternion (w, x, y, z)
 ****************************************************************************/

struct Quatf
{
    float w, x, y, z;

    Quatf() : w(1), x(0), y(0), z(0) {}
    Quatf(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    void normalize()
    {
        float n = sqrtf(w*w + x*x + y*y + z*z);
        if (n > 1e-6f) {
            float inv = 1.0f / n;
            w *= inv; x *= inv; y *= inv; z *= inv;
        }
    }

    Quatf conjugate() const { return {w, -x, -y, -z}; }

    // Quaternion nhân quaternion
    Quatf operator*(const Quatf& q) const
    {
        return {
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        };
    }

    // Xoay vector bằng quaternion: v' = q * [0,v] * q^-1
    Vector3f rotate(const Vector3f& v) const
    {
        float qwqw = w * w;
        float qwqx = w * x;
        float qwqy = w * y;
        float qwqz = w * z;
        float qxqx = x * x;
        float qxqy = x * y;
        float qxqz = x * z;
        float qyqy = y * y;
        float qyqz = y * z;
        float qzqz = z * z;

        return {
            v.x * (qwqw + qxqx - qyqy - qzqz) + 2.0f * (v.y * (qxqy - qwqz) + v.z * (qxqz + qwqy)),
            v.y * (qwqw - qxqx + qyqy - qzqz) + 2.0f * (v.x * (qxqy + qwqz) + v.z * (qyqz - qwqx)),
            v.z * (qwqw - qxqx - qyqy + qzqz) + 2.0f * (v.x * (qxqz - qwqy) + v.y * (qyqz + qwqx))
        };
    }

    // Xoay vector bằng quaternion nghịch đảo
    Vector3f rotate_inverse(const Vector3f& v) const
    {
        return conjugate().rotate(v);
    }

    // Tạo quaternion từ góc Euler (roll, pitch, yaw) ZYX convention
    static Quatf from_euler(float roll, float pitch, float yaw)
    {
        float cr = cosf(roll * 0.5f);
        float sr = sinf(roll * 0.5f);
        float cp = cosf(pitch * 0.5f);
        float sp = sinf(pitch * 0.5f);
        float cy = cosf(yaw * 0.5f);
        float sy = sinf(yaw * 0.5f);

        return {
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        };
    }

    // Lấy góc Euler từ quaternion
    void to_euler(float& roll, float& pitch, float& yaw) const
    {
        // Roll
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        roll = atan2f(sinr_cosp, cosr_cosp);

        // Pitch
        float sinp = 2.0f * (w * y - z * x);
        if (fabsf(sinp) >= 1.0f) {
            pitch = copysignf(1.5707963f, sinp); // ±π/2
        } else {
            pitch = asinf(sinp);
        }

        // Yaw
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        yaw = atan2f(siny_cosp, cosy_cosp);
    }

    // Truy cập theo index (w=0, x=1, y=2, z=3)
    float& operator[](int i) { return (&w)[i]; }
    float operator[](int i) const { return (&w)[i]; }
};

/****************************************************************************
 * Sample types - Dữ liệu từ các cảm biến
 ****************************************************************************/

// IMU sample (accelerometer + gyroscope)
struct ImuSample
{
    uint64_t timestamp_us;   // Thời gian lấy mẫu
    Vector3f accel;          // Gia tốc [m/s²]
    Vector3f gyro;           // Vận tốc góc [rad/s]
    float dt;                // Thời gian giữa 2 mẫu [s]
};

// GPS sample
struct GpsSample
{
    uint64_t timestamp_us;
    double lat;              // Vĩ độ [deg]
    double lon;              // Kinh độ [deg]
    float alt;               // Cao độ trên MSL [m]
    Vector3f vel;            // Vận tốc NED [m/s]
    float hacc;              // Độ chính xác ngang [m]
    float vacc;              // Độ chính xác đứng [m]
    float sacc;              // Độ chính xác vận tốc [m/s]
    uint8_t fix_type;        // Loại fix (0=no, 2=2D, 3=3D)
    uint8_t nsats;           // Số vệ tinh
};

// Magnetometer sample
struct MagSample
{
    uint64_t timestamp_us;
    Vector3f field;          // Từ trường [Gauss]
};

// Barometer sample
struct BaroSample
{
    uint64_t timestamp_us;
    float altitude;          // Cao độ từ baro [m]
    float pressure;          // Áp suất [Pa]
    float temperature;       // Nhiệt độ [°C]
};

/****************************************************************************
 * State sample - Trạng thái ước lượng
 ****************************************************************************/

struct StateSample
{
    Quatf quat;              // Tư thế quaternion (body to NED)
    Vector3f vel;            // Vận tốc NED [m/s]
    Vector3f pos;            // Vị trí NED [m]
    Vector3f gyro_bias;      // Bias gyro [rad/s]
    Vector3f accel_bias;     // Bias accel [m/s²]

    // Ghi vào mảng 1D
    void to_array(float out[STATE_SIZE]) const
    {
        out[0] = quat.w; out[1] = quat.x; out[2] = quat.y; out[3] = quat.z;
        out[4] = vel.x; out[5] = vel.y; out[6] = vel.z;
        out[7] = pos.x; out[8] = pos.y; out[9] = pos.z;
        out[10] = gyro_bias.x; out[11] = gyro_bias.y; out[12] = gyro_bias.z;
        out[13] = accel_bias.x; out[14] = accel_bias.y; out[15] = accel_bias.z;
    }

    // Đọc từ mảng 1D
    void from_array(const float in[STATE_SIZE])
    {
        quat.w = in[0]; quat.x = in[1]; quat.y = in[2]; quat.z = in[3];
        vel.x = in[4]; vel.y = in[5]; vel.z = in[6];
        pos.x = in[7]; pos.y = in[8]; pos.z = in[9];
        gyro_bias.x = in[10]; gyro_bias.y = in[11]; gyro_bias.z = in[12];
        accel_bias.x = in[13]; accel_bias.y = in[14]; accel_bias.z = in[15];
    }
};

/****************************************************************************
 * Tham số EKF2
 ****************************************************************************/

struct Parameters
{
    // IMU noise
    float gyro_noise{0.015f};         // Gyro noise [rad/s]
    float accel_noise{0.35f};         // Accel noise [m/s²]
    float gyro_bias_noise{0.001f};    // Gyro bias process noise [rad/s/√Hz]
    float accel_bias_noise{0.003f};   // Accel bias process noise [m/s²/√Hz]

    // Giới hạn bias
    float gyro_bias_lim{0.4f};        // Giới hạn gyro bias [rad/s]
    float accel_bias_lim{0.4f};       // Giới hạn accel bias [m/s²]

    // GPS fusion
    float gps_pos_noise{0.5f};        // GPS position noise [m]
    float gps_vel_noise{0.3f};        // GPS velocity noise [m/s]
    float gps_pos_gate{5.0f};         // Innovation gate [σ]
    float gps_vel_gate{5.0f};

    // Barometer fusion
    float baro_noise{2.0f};           // Baro altitude noise [m]
    float baro_gate{5.0f};

    // Magnetometer fusion
    float mag_noise{0.05f};           // Mag noise [Gauss]
    float mag_gate{3.0f};

    // Heading fusion
    float heading_noise{0.3f};        // Heading noise [rad]
    float heading_gate{2.6f};

    // Khởi tạo
    float init_tilt_err{0.1f};        // Initial tilt error [rad]
    float init_gyro_bias{0.1f};       // Initial gyro bias uncertainty [rad/s]
    float init_accel_bias{0.2f};      // Initial accel bias uncertainty [m/s²]

    // Gravity
    float gravity{9.80665f};
};

/****************************************************************************
 * Control status flags - Trạng thái điều khiển bộ lọc
 ****************************************************************************/

struct ControlStatus
{
    bool tilt_align : 1;          // Tư thế đã được khởi tạo từ accel
    bool yaw_align : 1;           // Yaw đã được khởi tạo (từ mag hoặc GPS)
    bool gps : 1;                 // Đang fusion GPS
    bool gps_hgt : 1;             // Đang dùng GPS cho độ cao
    bool baro_hgt : 1;            // Đang fusion baro cho độ cao
    bool mag_hdg : 1;             // Đang fusion mag heading
    bool mag_3d : 1;              // Đang fusion mag 3D
    bool in_air : 1;              // UAV đang bay
    bool wind : 1;                // Đang ước lượng gió
    bool vehicle_at_rest : 1;     // Phương tiện đứng yên
};

/****************************************************************************
 * Fault status flags - Trạng thái lỗi
 ****************************************************************************/

struct FaultStatus
{
    bool bad_mag_x : 1;
    bool bad_mag_y : 1;
    bool bad_mag_z : 1;
    bool bad_hdg : 1;
    bool bad_acc_bias : 1;
    bool bad_acc_vertical : 1;
};

/****************************************************************************
 * Innovation test ratios - Tỉ lệ kiểm tra innovation
 ****************************************************************************/

struct InnovationTestRatios
{
    float gps_hvel;      // GPS horizontal velocity
    float gps_vvel;      // GPS vertical velocity
    float gps_hpos;      // GPS horizontal position
    float gps_vpos;      // GPS vertical position
    float baro_hgt;      // Barometer height
    float mag_hdg;       // Magnetometer heading
};

} // namespace ekf2
