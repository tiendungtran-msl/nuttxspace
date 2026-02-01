/****************************************************************************
 * apps/examples/uav_states_v1/modules/estimator/attitude_module.cpp
 *
 * Triển khai module ước lượng tư thế
 *
 * Module này:
 * 1. Subscribe dữ liệu IMU từ ImuModule qua uORB topics.
 * 2. Chạy bộ lọc bổ sung quaternion (kiểu PX4).
 * 3. Publish vehicle_attitude cho hiển thị/điều khiển.
 *
 * Mỗi lần update():
 * - Kiểm tra tất cả subscription IMU có dữ liệu mới không.
 * - Với mỗi mẫu mới, chạy update estimator.
 * - Publish attitude đã cập nhật.
 *
 * Được gọi từ main loop (không tách thread riêng) để đơn giản.
 * Main loop điều khiển tốc độ cập nhật.
 *
 ****************************************************************************/

#include "attitude_module.hpp"
#include "../../platforms/nuttx/hrt/hrt.h"

#include <syslog.h>
#include <cstring>

namespace modules
{
namespace estimator
{

/****************************************************************************
 * Constructor
 ****************************************************************************/

AttitudeModule::AttitudeModule()
    : _num_estimators(0)
{
    memset(_imu_subs, 0, sizeof(_imu_subs));
    memset(_imu_topics, 0, sizeof(_imu_topics));
    memset(_last_imu_timestamp, 0, sizeof(_last_imu_timestamp));
    memset(_update_count, 0, sizeof(_update_count));
}

/****************************************************************************
 * init - Khởi tạo estimator và tạo subscription
 ****************************************************************************/

void AttitudeModule::init(uorb::Topic<uorb::sensor_imu_s, 8>* imu_topics, int num_imus)
{
    _num_estimators = (num_imus > MAX_ESTIMATORS) ? MAX_ESTIMATORS : num_imus;

    for (int i = 0; i < _num_estimators; i++) {
        // Lưu con trỏ topic và tạo subscription
        _imu_topics[i] = &imu_topics[i];
        _imu_subs[i] = new uorb::Subscription<uorb::sensor_imu_s, 8>(imu_topics[i]);

        // Khởi tạo estimator
        _estimators[i].reset();

        // Khởi tạo topic output
        _att_topics[i].init();
        _status_topics[i].init();

        _last_imu_timestamp[i] = 0;
        _update_count[i] = 0;
    }

    syslog(LOG_INFO, "[estimator] Initialized %d attitude estimators\n", _num_estimators);
}

/****************************************************************************
 * update - Xử lý một chu kỳ
 *
 * Với mỗi IMU có dữ liệu mới:
 * 1. Tính dt từ timestamp
 * 2. Cập nhật estimator quaternion
 * 3. Publish vehicle_attitude
 ****************************************************************************/

int AttitudeModule::update()
{
    int updated = 0;

    for (int i = 0; i < _num_estimators; i++) {
        if (!_imu_subs[i]) {
            continue;
        }

        uorb::sensor_imu_s imu;
        if (!_imu_subs[i]->copy(imu)) {
            continue;
        }

        // Tính dt từ timestamp
        float dt = 0.01f;  // Default 100Hz
        if (_last_imu_timestamp[i] > 0 && imu.timestamp_us > _last_imu_timestamp[i]) {
            dt = (imu.timestamp_us - _last_imu_timestamp[i]) * 1e-6f;
            // Giới hạn dt trong khoảng hợp lý
            if (dt < 0.001f) dt = 0.001f;
            if (dt > 0.1f) dt = 0.1f;
        }
        _last_imu_timestamp[i] = imu.timestamp_us;

        // Cập nhật estimator
        bool valid = _estimators[i].update(imu.accel, imu.gyro, dt);

        // Đóng gói và publish vehicle_attitude
        attitude::EulerAngles euler = _estimators[i].get_euler();
        attitude::Quatf q = _estimators[i].get_quaternion();
        attitude::Vector3f rates = _estimators[i].get_rates();

        uorb::vehicle_attitude_s att{};
        att.timestamp_us = hrt_absolute_time();
        att.q[0] = q.w;
        att.q[1] = q.x;
        att.q[2] = q.y;
        att.q[3] = q.z;
        att.roll = euler.roll;
        att.pitch = euler.pitch;
        att.yaw = euler.yaw;
        att.rollspeed = rates.x;
        att.pitchspeed = rates.y;
        att.yawspeed = rates.z;
        att.instance = static_cast<uint8_t>(i);

        _att_topics[i].publish(att);

        // Publish trạng thái estimator (debug/log)
        _update_count[i]++;
        if ((_update_count[i] % 100) == 0) {
            // Publish status mỗi 1 giây @ 100Hz
            attitude::Vector3f bias = _estimators[i].get_gyro_bias();

            uorb::estimator_status_s status{};
            status.timestamp_us = att.timestamp_us;
            status.gyro_bias[0] = bias.x;
            status.gyro_bias[1] = bias.y;
            status.gyro_bias[2] = bias.z;
            status.dt = dt;
            status.update_count = _update_count[i];
            status.instance = static_cast<uint8_t>(i);
            status.attitude_valid = valid;

            _status_topics[i].publish(status);
        }

        updated++;
    }

    return updated;
}

/****************************************************************************
 * get_attitude - Lấy attitude mới nhất (main dùng để hiển thị)
 ****************************************************************************/

bool AttitudeModule::get_attitude(int instance, uorb::vehicle_attitude_s& att)
{
    if (instance < 0 || instance >= _num_estimators) {
        return false;
    }
    return _att_topics[instance].copy(att);
}

} // namespace estimator
} // namespace modules
