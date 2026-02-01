/****************************************************************************
 * apps/examples/uav_states_v1/lib/ekf2/ekf2.hpp
 *
 * Extended Kalman Filter 2 - Lấy cảm hứng từ PX4 EKF2
 *
 * MỤC ĐÍCH:
 * - Ước lượng trạng thái đầy đủ của UAV: tư thế, vận tốc, vị trí.
 * - Fusion nhiều nguồn cảm biến: IMU, GPS, Mag, Baro.
 * - Ước lượng bias của gyro và accel online.
 *
 * THUẬT TOÁN:
 * 1. Prediction: Dùng IMU để dự đoán state và covariance.
 *    - Quaternion integration từ gyro (có bù bias).
 *    - Velocity integration từ accel (có bù bias, xoay về NED).
 *    - Position integration từ velocity.
 *
 * 2. Update: Dùng các cảm biến khác để hiệu chỉnh.
 *    - GPS: position và velocity.
 *    - Baro: altitude.
 *    - Mag: heading.
 *
 * STATE VECTOR (16 states):
 *   [0-3]   q = (qw, qx, qy, qz)  - Quaternion tư thế
 *   [4-6]   v = (vn, ve, vd)      - Vận tốc NED
 *   [7-9]   p = (pn, pe, pd)      - Vị trí NED
 *   [10-12] bg = (bgx, bgy, bgz)  - Bias gyro
 *   [13-15] ba = (bax, bay, baz)  - Bias accel
 *
 * LƯU Ý:
 * - Phiên bản đơn giản hóa so với PX4 (~25 states).
 * - Có thể mở rộng thêm mag_I, mag_B, wind sau này.
 *
 ****************************************************************************/

#pragma once

#include "ekf_types.hpp"
#include "ekf_matrix.hpp"

namespace ekf2
{

/****************************************************************************
 * Hằng số
 ****************************************************************************/

static constexpr float CONSTANTS_ONE_G = 9.80665f;
static constexpr float DT_MIN = 0.0001f;
static constexpr float DT_MAX = 0.1f;

/****************************************************************************
 * Ekf2 - Extended Kalman Filter class
 ****************************************************************************/

class Ekf2
{
public:
    Ekf2();
    ~Ekf2() = default;

    //=========================================================================
    // Khởi tạo và reset
    //=========================================================================

    /**
     * @brief Khởi tạo bộ lọc
     * @param timestamp_us Timestamp hiện tại [µs]
     * @return true nếu thành công
     */
    bool init(uint64_t timestamp_us);

    /**
     * @brief Reset về trạng thái ban đầu
     */
    void reset();

    //=========================================================================
    // Đầu vào cảm biến
    //=========================================================================

    /**
     * @brief Đưa dữ liệu IMU vào (gọi mỗi cycle @ 100-400Hz)
     *
     * Đây là hàm chính chạy prediction step.
     */
    void setImuData(const ImuSample& imu);

    /**
     * @brief Đưa dữ liệu GPS vào
     */
    void setGpsData(const GpsSample& gps);

    /**
     * @brief Đưa dữ liệu Magnetometer vào
     */
    void setMagData(const MagSample& mag);

    /**
     * @brief Đưa dữ liệu Barometer vào
     */
    void setBaroData(const BaroSample& baro);

    //=========================================================================
    // Chạy bộ lọc
    //=========================================================================

    /**
     * @brief Chạy một bước EKF (prediction + update)
     *
     * Gọi sau setImuData(). Tự động kiểm tra và fuse các sensor khác.
     *
     * @return true nếu attitude hợp lệ
     */
    bool update();

    //=========================================================================
    // Lấy kết quả
    //=========================================================================

    /**
     * @brief Lấy trạng thái ước lượng
     */
    const StateSample& getState() const { return _state; }

    /**
     * @brief Lấy quaternion tư thế
     */
    const Quatf& getQuaternion() const { return _state.quat; }

    /**
     * @brief Lấy góc Euler [rad]
     */
    void getEuler(float& roll, float& pitch, float& yaw) const
    {
        _state.quat.to_euler(roll, pitch, yaw);
    }

    /**
     * @brief Lấy vận tốc NED [m/s]
     */
    const Vector3f& getVelocity() const { return _state.vel; }

    /**
     * @brief Lấy vị trí NED [m]
     */
    const Vector3f& getPosition() const { return _state.pos; }

    /**
     * @brief Lấy bias gyro [rad/s]
     */
    const Vector3f& getGyroBias() const { return _state.gyro_bias; }

    /**
     * @brief Lấy bias accel [m/s²]
     */
    const Vector3f& getAccelBias() const { return _state.accel_bias; }

    /**
     * @brief Lấy control status
     */
    const ControlStatus& getControlStatus() const { return _control_status; }

    /**
     * @brief Lấy innovation test ratios
     */
    const InnovationTestRatios& getInnovTestRatios() const { return _innov_test_ratios; }

    //=========================================================================
    // Cấu hình
    //=========================================================================

    /**
     * @brief Lấy tham số (để chỉnh)
     */
    Parameters& getParams() { return _params; }
    const Parameters& getParams() const { return _params; }

    /**
     * @brief Đặt vị trí gốc (GPS origin)
     * @return true nếu thành công
     */
    bool setOrigin(double lat_deg, double lon_deg, float alt_m);

    /**
     * @brief Kiểm tra đã có origin chưa
     */
    bool hasOrigin() const { return _origin_set; }

private:
    //=========================================================================
    // Prediction step
    //=========================================================================

    /**
     * @brief Dự đoán state từ IMU
     */
    void predictState(const ImuSample& imu);

    /**
     * @brief Dự đoán covariance
     */
    void predictCovariance(const ImuSample& imu);

    /**
     * @brief Thêm process noise vào P
     */
    void addProcessNoise(float dt);

    //=========================================================================
    // Update steps (sensor fusion)
    //=========================================================================

    /**
     * @brief Fusion GPS velocity
     */
    void fuseGpsVel();

    /**
     * @brief Fusion GPS position
     */
    void fuseGpsPos();

    /**
     * @brief Fusion GPS altitude
     */
    void fuseGpsAlt();

    /**
     * @brief Fusion barometer altitude
     */
    void fuseBaroAlt();

    /**
     * @brief Fusion magnetometer heading
     */
    void fuseMagHeading();

    //=========================================================================
    // Kalman update helper
    //=========================================================================

    /**
     * @brief Kalman update cho một scalar observation
     *
     * @param H     Measurement Jacobian (1 x STATE_SIZE)
     * @param R     Measurement noise variance
     * @param innov Innovation (y - h(x))
     * @param gate  Innovation gate [σ]
     * @return true nếu fusion thành công
     */
    bool fuseScalar(const Vector<STATE_SIZE>& H, float R, float innov, float gate);

    /**
     * @brief Kiểm tra innovation có trong gate không
     */
    bool checkInnovationGate(float innov, float innov_var, float gate) const;

    //=========================================================================
    // Khởi tạo tư thế
    //=========================================================================

    /**
     * @brief Khởi tạo tilt từ accelerometer
     */
    bool initTiltFromAccel(const Vector3f& accel);

    /**
     * @brief Khởi tạo yaw từ magnetometer
     */
    bool initYawFromMag(const Vector3f& mag);

    //=========================================================================
    // Covariance maintenance
    //=========================================================================

    /**
     * @brief Giới hạn variance trong khoảng cho phép
     */
    void constrainVariances();

    /**
     * @brief Đảm bảo covariance đối xứng
     */
    void makeSymmetric();

    //=========================================================================
    // Utility
    //=========================================================================

    /**
     * @brief Chuyển GPS sang NED
     */
    void gpsToNed(double lat, double lon, float alt, Vector3f& pos_ned) const;

    /**
     * @brief Giới hạn giá trị
     */
    static float constrain(float val, float min_val, float max_val)
    {
        return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
    }

    //=========================================================================
    // State
    //=========================================================================

    StateSample _state;                              // Trạng thái ước lượng
    SquareMatrix<STATE_SIZE> _P;                     // Ma trận covariance

    //=========================================================================
    // Sensor data buffers (latest samples)
    //=========================================================================

    ImuSample _imu_sample;
    GpsSample _gps_sample;
    MagSample _mag_sample;
    BaroSample _baro_sample;

    bool _imu_updated{false};
    bool _gps_updated{false};
    bool _mag_updated{false};
    bool _baro_updated{false};

    //=========================================================================
    // Status
    //=========================================================================

    ControlStatus _control_status{};
    FaultStatus _fault_status{};
    InnovationTestRatios _innov_test_ratios{};

    //=========================================================================
    // Origin (GPS reference point)
    //=========================================================================

    double _origin_lat{0};
    double _origin_lon{0};
    float _origin_alt{0};
    bool _origin_set{false};

    //=========================================================================
    // Timing
    //=========================================================================

    uint64_t _time_us{0};
    uint64_t _time_last_imu{0};
    uint64_t _time_last_gps{0};
    uint64_t _time_last_mag{0};
    uint64_t _time_last_baro{0};

    //=========================================================================
    // Initialization
    //=========================================================================

    bool _filter_initialized{false};
    float _baro_alt_offset{0};              // Offset để baro = 0 lúc khởi động

    //=========================================================================
    // Parameters
    //=========================================================================

    Parameters _params;
};

} // namespace ekf2
