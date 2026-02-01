/****************************************************************************
 * apps/examples/uav_states_v1/lib/ekf2/ekf2.cpp
 *
 * Extended Kalman Filter 2 - Implementation
 *
 * Triển khai EKF2 với:
 * - Prediction từ IMU (gyro + accel)
 * - Update từ GPS, Baro, Mag
 * - Ước lượng bias online
 *
 ****************************************************************************/

#include "ekf2.hpp"
#include <cmath>
#include <cstring>

namespace ekf2
{

/****************************************************************************
 * Constructor
 ****************************************************************************/

Ekf2::Ekf2()
{
    reset();
}

/****************************************************************************
 * reset - Reset về trạng thái ban đầu
 ****************************************************************************/

void Ekf2::reset()
{
    // Reset state
    _state.quat = Quatf();  // Identity quaternion
    _state.vel.zero();
    _state.pos.zero();
    _state.gyro_bias.zero();
    _state.accel_bias.zero();

    // Reset covariance - large initial uncertainty
    _P.zero();

    // Quaternion uncertainty (tilt)
    float tilt_var = _params.init_tilt_err * _params.init_tilt_err;
    _P(0, 0) = 1.0f;  // qw - nhỏ vì qw ≈ 1
    _P(1, 1) = tilt_var;
    _P(2, 2) = tilt_var;
    _P(3, 3) = 1.0f;  // yaw - lớn vì chưa biết

    // Velocity uncertainty
    float vel_var = 1.0f;
    _P(4, 4) = vel_var;
    _P(5, 5) = vel_var;
    _P(6, 6) = vel_var;

    // Position uncertainty
    float pos_var = 10.0f;
    _P(7, 7) = pos_var;
    _P(8, 8) = pos_var;
    _P(9, 9) = pos_var;

    // Gyro bias uncertainty
    float gbias_var = _params.init_gyro_bias * _params.init_gyro_bias;
    _P(10, 10) = gbias_var;
    _P(11, 11) = gbias_var;
    _P(12, 12) = gbias_var;

    // Accel bias uncertainty
    float abias_var = _params.init_accel_bias * _params.init_accel_bias;
    _P(13, 13) = abias_var;
    _P(14, 14) = abias_var;
    _P(15, 15) = abias_var;

    // Reset flags
    memset(&_control_status, 0, sizeof(_control_status));
    memset(&_fault_status, 0, sizeof(_fault_status));
    memset(&_innov_test_ratios, 0, sizeof(_innov_test_ratios));

    _imu_updated = false;
    _gps_updated = false;
    _mag_updated = false;
    _baro_updated = false;

    _filter_initialized = false;
    _origin_set = false;
    _baro_alt_offset = 0;

    _time_us = 0;
    _time_last_imu = 0;
    _time_last_gps = 0;
    _time_last_mag = 0;
    _time_last_baro = 0;
}

/****************************************************************************
 * init - Khởi tạo bộ lọc
 ****************************************************************************/

bool Ekf2::init(uint64_t timestamp_us)
{
    reset();
    _time_us = timestamp_us;
    _time_last_imu = timestamp_us;
    return true;
}

/****************************************************************************
 * setImuData - Nhận dữ liệu IMU
 ****************************************************************************/

void Ekf2::setImuData(const ImuSample& imu)
{
    _imu_sample = imu;
    _imu_updated = true;

    // Tính dt từ timestamp
    if (_time_last_imu > 0 && imu.timestamp_us > _time_last_imu) {
        _imu_sample.dt = (imu.timestamp_us - _time_last_imu) * 1e-6f;
    } else {
        _imu_sample.dt = 0.01f;  // Default 100Hz
    }

    // Giới hạn dt
    _imu_sample.dt = constrain(_imu_sample.dt, DT_MIN, DT_MAX);

    _time_last_imu = imu.timestamp_us;
    _time_us = imu.timestamp_us;
}

/****************************************************************************
 * setGpsData - Nhận dữ liệu GPS
 ****************************************************************************/

void Ekf2::setGpsData(const GpsSample& gps)
{
    _gps_sample = gps;
    _gps_updated = true;
    _time_last_gps = gps.timestamp_us;

    // Tự động set origin nếu chưa có và GPS tốt
    if (!_origin_set && gps.fix_type >= 3 && gps.nsats >= 6) {
        setOrigin(gps.lat, gps.lon, gps.alt);
    }
}

/****************************************************************************
 * setMagData - Nhận dữ liệu Mag
 ****************************************************************************/

void Ekf2::setMagData(const MagSample& mag)
{
    _mag_sample = mag;
    _mag_updated = true;
    _time_last_mag = mag.timestamp_us;
}

/****************************************************************************
 * setBaroData - Nhận dữ liệu Baro
 ****************************************************************************/

void Ekf2::setBaroData(const BaroSample& baro)
{
    _baro_sample = baro;
    _baro_updated = true;
    _time_last_baro = baro.timestamp_us;

    // Set offset lần đầu để baro = 0
    if (_baro_alt_offset == 0 && baro.altitude != 0) {
        _baro_alt_offset = baro.altitude;
    }
}

/****************************************************************************
 * setOrigin - Đặt điểm gốc GPS
 ****************************************************************************/

bool Ekf2::setOrigin(double lat_deg, double lon_deg, float alt_m)
{
    _origin_lat = lat_deg;
    _origin_lon = lon_deg;
    _origin_alt = alt_m;
    _origin_set = true;
    return true;
}

/****************************************************************************
 * update - Chạy một bước EKF
 ****************************************************************************/

bool Ekf2::update()
{
    if (!_imu_updated) {
        return _control_status.tilt_align;
    }

    //=========================================================================
    // Khởi tạo tư thế từ cảm biến nếu chưa
    //=========================================================================

    if (!_control_status.tilt_align) {
        // Khởi tạo tilt từ accel (UAV đang đứng yên)
        if (initTiltFromAccel(_imu_sample.accel)) {
            _control_status.tilt_align = true;
        }
    }

    if (!_control_status.yaw_align && _mag_updated) {
        // Khởi tạo yaw từ mag
        if (initYawFromMag(_mag_sample.field)) {
            _control_status.yaw_align = true;
        }
    }

    //=========================================================================
    // PREDICTION STEP
    // Dự đoán state và covariance từ IMU
    //=========================================================================

    predictState(_imu_sample);
    predictCovariance(_imu_sample);

    //=========================================================================
    // UPDATE STEPS
    // Fusion các cảm biến khác
    //=========================================================================

    // GPS fusion
    if (_gps_updated && _origin_set && _gps_sample.fix_type >= 3) {
        fuseGpsVel();
        fuseGpsPos();
        _control_status.gps = true;
        _gps_updated = false;
    }

    // Baro fusion
    if (_baro_updated) {
        fuseBaroAlt();
        _control_status.baro_hgt = true;
        _baro_updated = false;
    }

    // Mag heading fusion
    if (_mag_updated && _control_status.tilt_align) {
        fuseMagHeading();
        _control_status.mag_hdg = true;
        _mag_updated = false;
    }

    //=========================================================================
    // Covariance maintenance
    //=========================================================================

    constrainVariances();
    makeSymmetric();

    _imu_updated = false;
    _filter_initialized = true;

    return _control_status.tilt_align;
}

/****************************************************************************
 * initTiltFromAccel - Khởi tạo roll/pitch từ accel
 *
 * Khi UAV đứng yên, accel đo chỉ có gravity.
 * Dùng đó để tính roll và pitch ban đầu.
 ****************************************************************************/

bool Ekf2::initTiltFromAccel(const Vector3f& accel)
{
    float accel_norm = accel.norm();

    // Kiểm tra accel hợp lý (gần 1g)
    if (accel_norm < 0.5f * _params.gravity || accel_norm > 1.5f * _params.gravity) {
        return false;
    }

    // Tính roll, pitch từ accel
    // Giả sử yaw = 0
    float roll = atan2f(accel.y, accel.z);
    float pitch = atan2f(-accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z));

    _state.quat = Quatf::from_euler(roll, pitch, 0.0f);

    return true;
}

/****************************************************************************
 * initYawFromMag - Khởi tạo yaw từ magnetometer
 *
 * Dùng mag projection lên mặt phẳng ngang để tính heading.
 ****************************************************************************/

bool Ekf2::initYawFromMag(const Vector3f& mag)
{
    // Xoay mag về body frame đã có tilt
    // Tính heading trong NED frame

    float mag_norm = mag.norm();
    if (mag_norm < 0.1f) {
        return false;
    }

    // Xoay mag về NED frame (dùng tilt đã có)
    Vector3f mag_ned = _state.quat.rotate(mag);

    // Heading = atan2(East, North)
    float yaw = atan2f(mag_ned.y, mag_ned.x);

    // Cập nhật quaternion với yaw mới
    float roll, pitch, old_yaw;
    _state.quat.to_euler(roll, pitch, old_yaw);
    _state.quat = Quatf::from_euler(roll, pitch, yaw);

    return true;
}

/****************************************************************************
 * predictState - Dự đoán state từ IMU
 *
 * THUẬT TOÁN:
 * 1. Bù bias cho gyro và accel
 * 2. Integrate quaternion: q = q + 0.5 * q * omega * dt
 * 3. Xoay accel về NED, trừ gravity
 * 4. Integrate velocity: v = v + a * dt
 * 5. Integrate position: p = p + v * dt
 ****************************************************************************/

void Ekf2::predictState(const ImuSample& imu)
{
    float dt = imu.dt;

    //-------------------------------------------------------------------------
    // 1. Bù bias
    //-------------------------------------------------------------------------

    Vector3f gyro_corrected = imu.gyro - _state.gyro_bias;
    Vector3f accel_corrected = imu.accel - _state.accel_bias;

    //-------------------------------------------------------------------------
    // 2. Quaternion integration (1st order)
    //
    // q_dot = 0.5 * q * [0, omega]
    // q_new = q + q_dot * dt
    //-------------------------------------------------------------------------

    float omega_norm = gyro_corrected.norm();

    if (omega_norm > 1e-6f) {
        // Rodrigues rotation
        float half_angle = 0.5f * omega_norm * dt;
        float sin_half = sinf(half_angle);
        float cos_half = cosf(half_angle);

        Vector3f axis = gyro_corrected * (1.0f / omega_norm);

        Quatf delta_q(
            cos_half,
            axis.x * sin_half,
            axis.y * sin_half,
            axis.z * sin_half
        );

        _state.quat = _state.quat * delta_q;
        _state.quat.normalize();
    }

    //-------------------------------------------------------------------------
    // 3. Xoay accel về NED frame và trừ gravity
    //-------------------------------------------------------------------------

    Vector3f accel_ned = _state.quat.rotate(accel_corrected);
    accel_ned.z += _params.gravity;  // Gravity down = positive z

    //-------------------------------------------------------------------------
    // 4. Velocity integration
    //-------------------------------------------------------------------------

    _state.vel.x += accel_ned.x * dt;
    _state.vel.y += accel_ned.y * dt;
    _state.vel.z += accel_ned.z * dt;

    //-------------------------------------------------------------------------
    // 5. Position integration
    //-------------------------------------------------------------------------

    _state.pos.x += _state.vel.x * dt;
    _state.pos.y += _state.vel.y * dt;
    _state.pos.z += _state.vel.z * dt;
}

/****************************************************************************
 * predictCovariance - Dự đoán covariance
 *
 * P = F * P * F' + Q
 *
 * Trong đó F là state transition Jacobian.
 * Đơn giản hóa: chỉ cập nhật các block chính.
 ****************************************************************************/

void Ekf2::predictCovariance(const ImuSample& imu)
{
    float dt = imu.dt;
    float dt2 = dt * dt;

    //-------------------------------------------------------------------------
    // Thêm process noise Q
    //-------------------------------------------------------------------------

    addProcessNoise(dt);

    //-------------------------------------------------------------------------
    // Propagate quaternion covariance
    // (đơn giản hóa: chỉ thêm gyro noise)
    //-------------------------------------------------------------------------

    float gyro_var = _params.gyro_noise * _params.gyro_noise * dt2;
    _P(1, 1) += gyro_var;
    _P(2, 2) += gyro_var;
    _P(3, 3) += gyro_var;

    //-------------------------------------------------------------------------
    // Propagate velocity covariance
    // vel depends on accel -> add accel noise
    //-------------------------------------------------------------------------

    float accel_var = _params.accel_noise * _params.accel_noise * dt2;
    _P(4, 4) += accel_var;
    _P(5, 5) += accel_var;
    _P(6, 6) += accel_var;

    //-------------------------------------------------------------------------
    // Propagate position covariance
    // pos depends on vel
    //-------------------------------------------------------------------------

    float vel_var_contribution = dt2 * (_P(4, 4) + _P(5, 5) + _P(6, 6)) / 3.0f;
    _P(7, 7) += vel_var_contribution + _P(4, 4) * dt2;
    _P(8, 8) += vel_var_contribution + _P(5, 5) * dt2;
    _P(9, 9) += vel_var_contribution + _P(6, 6) * dt2;
}

/****************************************************************************
 * addProcessNoise - Thêm process noise vào covariance
 ****************************************************************************/

void Ekf2::addProcessNoise(float dt)
{
    // Gyro bias random walk
    float gbias_var = _params.gyro_bias_noise * _params.gyro_bias_noise * dt;
    _P(10, 10) += gbias_var;
    _P(11, 11) += gbias_var;
    _P(12, 12) += gbias_var;

    // Accel bias random walk
    float abias_var = _params.accel_bias_noise * _params.accel_bias_noise * dt;
    _P(13, 13) += abias_var;
    _P(14, 14) += abias_var;
    _P(15, 15) += abias_var;
}

/****************************************************************************
 * fuseGpsVel - Fusion GPS velocity
 *
 * GPS đo velocity trực tiếp (Doppler).
 * H = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] cho Vn
 ****************************************************************************/

void Ekf2::fuseGpsVel()
{
    if (_gps_sample.sacc > 5.0f) {
        // GPS velocity quá kém
        return;
    }

    float R = _params.gps_vel_noise * _params.gps_vel_noise;
    R += _gps_sample.sacc * _gps_sample.sacc;

    // Fuse Vn
    {
        Vector<STATE_SIZE> H;
        H.zero();
        H[StateIdx::VEL + 0] = 1.0f;

        float innov = _state.vel.x - _gps_sample.vel.x;
        fuseScalar(H, R, innov, _params.gps_vel_gate);
    }

    // Fuse Ve
    {
        Vector<STATE_SIZE> H;
        H.zero();
        H[StateIdx::VEL + 1] = 1.0f;

        float innov = _state.vel.y - _gps_sample.vel.y;
        fuseScalar(H, R, innov, _params.gps_vel_gate);
    }

    // Fuse Vd
    {
        Vector<STATE_SIZE> H;
        H.zero();
        H[StateIdx::VEL + 2] = 1.0f;

        float innov = _state.vel.z - _gps_sample.vel.z;
        fuseScalar(H, R, innov, _params.gps_vel_gate);
    }
}

/****************************************************************************
 * fuseGpsPos - Fusion GPS horizontal position
 ****************************************************************************/

void Ekf2::fuseGpsPos()
{
    if (_gps_sample.hacc > 10.0f || !_origin_set) {
        return;
    }

    // Chuyển GPS sang NED
    Vector3f gps_ned;
    gpsToNed(_gps_sample.lat, _gps_sample.lon, _gps_sample.alt, gps_ned);

    float R = _params.gps_pos_noise * _params.gps_pos_noise;
    R += _gps_sample.hacc * _gps_sample.hacc;

    // Fuse Pn
    {
        Vector<STATE_SIZE> H;
        H.zero();
        H[StateIdx::POS + 0] = 1.0f;

        float innov = _state.pos.x - gps_ned.x;
        _innov_test_ratios.gps_hpos = innov * innov / R;
        fuseScalar(H, R, innov, _params.gps_pos_gate);
    }

    // Fuse Pe
    {
        Vector<STATE_SIZE> H;
        H.zero();
        H[StateIdx::POS + 1] = 1.0f;

        float innov = _state.pos.y - gps_ned.y;
        fuseScalar(H, R, innov, _params.gps_pos_gate);
    }

    // Fuse Pd (altitude)
    {
        float R_alt = _params.gps_pos_noise * _params.gps_pos_noise;
        R_alt += _gps_sample.vacc * _gps_sample.vacc;

        Vector<STATE_SIZE> H;
        H.zero();
        H[StateIdx::POS + 2] = 1.0f;

        float innov = _state.pos.z - gps_ned.z;
        _innov_test_ratios.gps_vpos = innov * innov / R_alt;
        fuseScalar(H, R_alt, innov, _params.gps_pos_gate);
    }
}

/****************************************************************************
 * fuseBaroAlt - Fusion barometer altitude
 *
 * Baro đo relative altitude (có offset).
 ****************************************************************************/

void Ekf2::fuseBaroAlt()
{
    float baro_alt = _baro_sample.altitude - _baro_alt_offset;

    // Baro đo altitude down = -pos_z
    float predicted = -_state.pos.z;

    float R = _params.baro_noise * _params.baro_noise;

    Vector<STATE_SIZE> H;
    H.zero();
    H[StateIdx::POS + 2] = -1.0f;  // d(baro)/d(pos_z) = -1

    float innov = predicted - baro_alt;
    _innov_test_ratios.baro_hgt = innov * innov / R;

    fuseScalar(H, R, innov, _params.baro_gate);
}

/****************************************************************************
 * fuseMagHeading - Fusion magnetometer heading
 *
 * Dùng mag để hiệu chỉnh yaw (heading).
 ****************************************************************************/

void Ekf2::fuseMagHeading()
{
    // Xoay mag về NED
    Vector3f mag_ned = _state.quat.rotate(_mag_sample.field);

    // Tính heading từ mag
    float mag_heading = atan2f(mag_ned.y, mag_ned.x);

    // Lấy heading hiện tại từ quaternion
    float roll, pitch, yaw;
    _state.quat.to_euler(roll, pitch, yaw);

    // Innovation
    float innov = yaw - mag_heading;

    // Wrap to [-π, π]
    while (innov > 3.14159265f) innov -= 6.28318530f;
    while (innov < -3.14159265f) innov += 6.28318530f;

    float R = _params.heading_noise * _params.heading_noise;
    _innov_test_ratios.mag_hdg = innov * innov / R;

    // H matrix cho heading (ảnh hưởng quat)
    // Đơn giản hóa: chỉ cập nhật q3 (yaw component)
    Vector<STATE_SIZE> H;
    H.zero();
    H[3] = 1.0f;  // qz component

    // Apply update nếu trong gate
    if (checkInnovationGate(innov, R + _P(3, 3), _params.heading_gate)) {
        // Đơn giản: chỉ cập nhật yaw trong quaternion
        _state.quat = Quatf::from_euler(roll, pitch, yaw - 0.1f * innov);
        _state.quat.normalize();
    }
}

/****************************************************************************
 * fuseScalar - Kalman update cho scalar observation
 *
 * Standard Kalman update:
 *   S = H * P * H' + R
 *   K = P * H' / S
 *   x = x - K * innov
 *   P = (I - K * H) * P
 ****************************************************************************/

bool Ekf2::fuseScalar(const Vector<STATE_SIZE>& H, float R, float innov, float gate)
{
    // Tính innovation variance: S = H * P * H' + R
    float S = R;
    for (int i = 0; i < STATE_SIZE; i++) {
        if (H[i] != 0) {
            for (int j = 0; j < STATE_SIZE; j++) {
                if (H[j] != 0) {
                    S += H[i] * _P(i, j) * H[j];
                }
            }
        }
    }

    // Kiểm tra innovation gate
    if (!checkInnovationGate(innov, S, gate)) {
        return false;
    }

    // Tính Kalman gain: K = P * H' / S
    Vector<STATE_SIZE> K;
    float S_inv = 1.0f / S;
    for (int i = 0; i < STATE_SIZE; i++) {
        float sum = 0;
        for (int j = 0; j < STATE_SIZE; j++) {
            if (H[j] != 0) {
                sum += _P(i, j) * H[j];
            }
        }
        K[i] = sum * S_inv;
    }

    // Cập nhật state: x = x - K * innov
    float state_array[STATE_SIZE];
    _state.to_array(state_array);

    for (int i = 0; i < STATE_SIZE; i++) {
        state_array[i] -= K[i] * innov;
    }

    _state.from_array(state_array);

    // Normalize quaternion
    _state.quat.normalize();

    // Giới hạn bias
    _state.gyro_bias.x = constrain(_state.gyro_bias.x, -_params.gyro_bias_lim, _params.gyro_bias_lim);
    _state.gyro_bias.y = constrain(_state.gyro_bias.y, -_params.gyro_bias_lim, _params.gyro_bias_lim);
    _state.gyro_bias.z = constrain(_state.gyro_bias.z, -_params.gyro_bias_lim, _params.gyro_bias_lim);
    _state.accel_bias.x = constrain(_state.accel_bias.x, -_params.accel_bias_lim, _params.accel_bias_lim);
    _state.accel_bias.y = constrain(_state.accel_bias.y, -_params.accel_bias_lim, _params.accel_bias_lim);
    _state.accel_bias.z = constrain(_state.accel_bias.z, -_params.accel_bias_lim, _params.accel_bias_lim);

    // Cập nhật covariance: P = (I - K * H) * P
    // Joseph form cho numerical stability: P = (I - K*H) * P * (I - K*H)' + K*R*K'
    // Đơn giản hóa: P = P - K * S * K'
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j <= i; j++) {
            float delta = K[i] * S * K[j];
            _P(i, j) -= delta;
            if (i != j) {
                _P(j, i) -= delta;
            }
        }
    }

    return true;
}

/****************************************************************************
 * checkInnovationGate - Kiểm tra innovation có trong gate không
 ****************************************************************************/

bool Ekf2::checkInnovationGate(float innov, float innov_var, float gate) const
{
    float test_ratio = innov * innov / innov_var;
    return test_ratio < gate * gate;
}

/****************************************************************************
 * constrainVariances - Giới hạn variance
 ****************************************************************************/

void Ekf2::constrainVariances()
{
    // Quaternion variance
    for (int i = 0; i < 4; i++) {
        _P.constrainVariance(i, 0.0f, 1.0f);
    }

    // Velocity variance
    for (int i = 4; i < 7; i++) {
        _P.constrainVariance(i, 1e-6f, 1e4f);
    }

    // Position variance
    for (int i = 7; i < 10; i++) {
        _P.constrainVariance(i, 1e-6f, 1e6f);
    }

    // Bias variance
    float max_bias_var = 0.1f;
    for (int i = 10; i < 16; i++) {
        _P.constrainVariance(i, 1e-8f, max_bias_var);
    }
}

/****************************************************************************
 * makeSymmetric - Đảm bảo covariance đối xứng
 ****************************************************************************/

void Ekf2::makeSymmetric()
{
    _P.makeSymmetric();
}

/****************************************************************************
 * gpsToNed - Chuyển GPS lat/lon/alt sang NED
 *
 * Dùng simple flat-earth approximation gần origin.
 ****************************************************************************/

void Ekf2::gpsToNed(double lat, double lon, float alt, Vector3f& pos_ned) const
{
    if (!_origin_set) {
        pos_ned.zero();
        return;
    }

    // Earth radius
    static constexpr double R_EARTH = 6371000.0;
    static constexpr double DEG_TO_RAD = 0.017453292519943295;

    double dlat = (lat - _origin_lat) * DEG_TO_RAD;
    double dlon = (lon - _origin_lon) * DEG_TO_RAD;
    double lat_rad = _origin_lat * DEG_TO_RAD;

    // North
    pos_ned.x = static_cast<float>(dlat * R_EARTH);

    // East
    pos_ned.y = static_cast<float>(dlon * R_EARTH * cos(lat_rad));

    // Down (positive down)
    pos_ned.z = _origin_alt - alt;
}

} // namespace ekf2
