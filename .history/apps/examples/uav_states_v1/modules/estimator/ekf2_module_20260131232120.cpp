/****************************************************************************
 * apps/examples/uav_states_v1/modules/estimator/ekf2_module.cpp
 *
 * Module EKF2 - Implementation
 *
 * Triển khai EKF2 module để fusion cảm biến:
 * - IMU: prediction step
 * - GPS: position/velocity update
 * - Mag: heading update
 * - Baro: altitude update
 *
 ****************************************************************************/

#include "ekf2_module.hpp"
#include <cstring>
#include <cmath>

namespace modules
{
namespace estimator
{

/****************************************************************************
 * Constructor
 ****************************************************************************/

Ekf2Module::Ekf2Module()
    : _imu_topic(nullptr)
    , _imu_sub(nullptr)
    , _filter_init(false)
    , _update_count(0)
{
    memset(&_last_local_pos, 0, sizeof(_last_local_pos));
    memset(&_last_attitude, 0, sizeof(_last_attitude));
}

/****************************************************************************
 * init - Khởi tạo EKF2 module
 ****************************************************************************/

void Ekf2Module::init(uorb::Topic<uorb::sensor_imu_s, 8>* imu_topic)
{
    _imu_topic = imu_topic;

    // Tạo subscription cho IMU topic
    if (_imu_topic) {
        static uorb::Subscription<uorb::sensor_imu_s, 8> sub(*_imu_topic);
        _imu_sub = &sub;
    }

    // Lấy timestamp hiện tại (giả sử từ system)
    uint64_t now_us = 0;
    if (_imu_topic && _imu_sub) {
        uorb::sensor_imu_s imu;
        if (_imu_sub->copy_force(imu)) {
            now_us = imu.timestamp_us;
        }
    }

    // Khởi tạo EKF2
    _ekf.init(now_us);
    _filter_init = true;
}

/****************************************************************************
 * update - Xử lý một chu kỳ
 ****************************************************************************/

bool Ekf2Module::update()
{
    if (!_imu_topic || !_imu_sub) {
        return false;
    }

    // Kiểm tra có dữ liệu IMU mới
    if (!_imu_sub->updated()) {
        return false;
    }

    // Đọc IMU sample
    uorb::sensor_imu_s imu;
    if (!_imu_sub->copy(imu)) {
        return false;
    }

    //=========================================================================
    // Chuyển dữ liệu IMU sang format EKF2
    //=========================================================================

    ekf2::ImuSample imu_sample;
    imu_sample.timestamp_us = imu.timestamp_us;
    imu_sample.gyro.x = imu.gyro[0];
    imu_sample.gyro.y = imu.gyro[1];
    imu_sample.gyro.z = imu.gyro[2];
    imu_sample.accel.x = imu.accel[0];
    imu_sample.accel.y = imu.accel[1];
    imu_sample.accel.z = imu.accel[2];

    // Gửi IMU vào EKF
    _ekf.setImuData(imu_sample);

    //=========================================================================
    // Chạy EKF update
    //=========================================================================

    bool updated = _ekf.update();

    if (updated) {
        _update_count++;

        // Publish kết quả
        publishAttitude(imu.timestamp_us);
        publishLocalPosition(imu.timestamp_us);

        // Publish status mỗi 10 lần
        if (_update_count % 10 == 0) {
            publishStatus(imu.timestamp_us);
        }
    }

    return updated;
}

/****************************************************************************
 * setGpsData - Nhận dữ liệu GPS
 ****************************************************************************/

void Ekf2Module::setGpsData(const uorb::sensor_gps_s& gps)
{
    ekf2::GpsSample sample;
    sample.timestamp_us = gps.timestamp_us;
    sample.lat = gps.lat;
    sample.lon = gps.lon;
    sample.alt = gps.alt;
    sample.vel.x = gps.vel_n;
    sample.vel.y = gps.vel_e;
    sample.vel.z = gps.vel_d;
    sample.hacc = gps.hacc;
    sample.vacc = gps.vacc;
    sample.sacc = gps.sacc;
    sample.fix_type = gps.fix_type;
    sample.nsats = gps.nsats;

    _ekf.setGpsData(sample);
}

/****************************************************************************
 * setMagData - Nhận dữ liệu Magnetometer
 ****************************************************************************/

void Ekf2Module::setMagData(const uorb::sensor_mag_s& mag)
{
    ekf2::MagSample sample;
    sample.timestamp_us = mag.timestamp_us;
    sample.field.x = mag.field[0];
    sample.field.y = mag.field[1];
    sample.field.z = mag.field[2];

    _ekf.setMagData(sample);
}

/****************************************************************************
 * setBaroData - Nhận dữ liệu Barometer
 ****************************************************************************/

void Ekf2Module::setBaroData(const uorb::sensor_baro_s& baro)
{
    ekf2::BaroSample sample;
    sample.timestamp_us = baro.timestamp_us;
    sample.altitude = baro.altitude;
    sample.pressure = baro.pressure;
    sample.temperature = baro.temperature;

    _ekf.setBaroData(sample);
}

/****************************************************************************
 * publishAttitude - Publish attitude topic
 ****************************************************************************/

void Ekf2Module::publishAttitude(uint64_t timestamp)
{
    const ekf2::StateSample& state = _ekf.getState();

    uorb::vehicle_attitude_s att;
    att.timestamp_us = timestamp;

    // Quaternion
    att.q[0] = state.quat.w;
    att.q[1] = state.quat.x;
    att.q[2] = state.quat.y;
    att.q[3] = state.quat.z;

    // Euler angles
    state.quat.to_euler(att.roll, att.pitch, att.yaw);

    // Angular rates (từ gyro đã bù bias)
    // Tạm thời dùng 0, có thể lấy từ gyro sample
    att.rollspeed = 0;
    att.pitchspeed = 0;
    att.yawspeed = 0;

    att.instance = 0;

    // Publish
    _attitude_topic.publish(att);
    _last_attitude = att;
}

/****************************************************************************
 * publishLocalPosition - Publish local position topic
 ****************************************************************************/

void Ekf2Module::publishLocalPosition(uint64_t timestamp)
{
    const ekf2::StateSample& state = _ekf.getState();
    const ekf2::ControlStatus& status = _ekf.getControlStatus();

    uorb::vehicle_local_position_s pos;
    memset(&pos, 0, sizeof(pos));

    pos.timestamp_us = timestamp;

    // Position NED
    pos.x = state.pos.x;
    pos.y = state.pos.y;
    pos.z = state.pos.z;

    // Velocity NED
    pos.vx = state.vel.x;
    pos.vy = state.vel.y;
    pos.vz = state.vel.z;

    // Origin
    double origin_lat, origin_lon;
    float origin_alt;
    if (_ekf.getOrigin(origin_lat, origin_lon, origin_alt)) {
        pos.ref_lat = origin_lat;
        pos.ref_lon = origin_lon;
        pos.ref_alt = origin_alt;
    }

    // Validity flags
    pos.xy_valid = status.gps;
    pos.z_valid = status.baro_hgt;
    pos.v_xy_valid = status.gps;
    pos.v_z_valid = true;

    // Publish
    _local_pos_topic.publish(pos);
    _last_local_pos = pos;
}

/****************************************************************************
 * publishStatus - Publish EKF2 status topic
 ****************************************************************************/

void Ekf2Module::publishStatus(uint64_t timestamp)
{
    const ekf2::StateSample& state = _ekf.getState();
    const ekf2::ControlStatus& ctrl = _ekf.getControlStatus();
    const ekf2::InnovationTestRatios& innov = _ekf.getInnovTestRatios();

    uorb::ekf2_status_s status;
    memset(&status, 0, sizeof(status));

    status.timestamp_us = timestamp;

    // Bias estimates
    status.gyro_bias[0] = state.gyro_bias.x;
    status.gyro_bias[1] = state.gyro_bias.y;
    status.gyro_bias[2] = state.gyro_bias.z;
    status.accel_bias[0] = state.accel_bias.x;
    status.accel_bias[1] = state.accel_bias.y;
    status.accel_bias[2] = state.accel_bias.z;

    // Innovation test ratios
    status.gps_hpos_test_ratio = innov.gps_hpos;
    status.gps_vpos_test_ratio = innov.gps_vpos;
    status.baro_hgt_test_ratio = innov.baro_hgt;
    status.mag_heading_test_ratio = innov.mag_hdg;

    // Control status
    status.tilt_align = ctrl.tilt_align;
    status.yaw_align = ctrl.yaw_align;
    status.gps_fused = ctrl.gps;
    status.baro_fused = ctrl.baro_hgt;
    status.mag_fused = ctrl.mag_hdg;

    // Publish
    _status_topic.publish(status);
}

/****************************************************************************
 * get_attitude - Lấy attitude mới nhất
 ****************************************************************************/

bool Ekf2Module::get_attitude(uorb::vehicle_attitude_s& att)
{
    if (!_ekf.isInitialized()) {
        return false;
    }

    att = _last_attitude;
    return true;
}

/****************************************************************************
 * get_local_position - Lấy local position mới nhất
 ****************************************************************************/

bool Ekf2Module::get_local_position(uorb::vehicle_local_position_s& pos)
{
    if (!_ekf.isInitialized()) {
        return false;
    }

    pos = _last_local_pos;
    return true;
}

} // namespace estimator
} // namespace modules
