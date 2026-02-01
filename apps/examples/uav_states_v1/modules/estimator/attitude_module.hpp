/****************************************************************************
 * apps/examples/uav_states_v1/modules/estimator/attitude_module.hpp
 *
 * Module ước lượng tư thế - Dùng quaternion
 *
 * MỤC ĐÍCH:
 * - Nhận dữ liệu IMU từ topic sensor_imu (CONSUMER).
 * - Tính tư thế bằng bộ lọc bổ sung quaternion.
 * - Publish topic vehicle_attitude (PRODUCER).
 *
 * KIẾN TRÚC:
 * - Mỗi IMU có một estimator để dự phòng.
 * - Subscribe sensor_imu từ ImuModule.
 * - Publish vehicle_attitude cho hiển thị/điều khiển.
 * - Tương lai: voting cảm biến, tích hợp EKF2.
 *
 * LUỒNG DỮ LIỆU:
 *   ImuModule (sensor_imu) --> AttitudeModule --> (vehicle_attitude) --> Main
 *
 ****************************************************************************/

#pragma once

#include <cstdint>

#include "../../uorb/uorb.hpp"
#include "../../uorb/topics.hpp"
#include "../../lib/attitude_estimator/attitude_estimator_q.hpp"

namespace modules
{
namespace estimator
{

/****************************************************************************
 * Cấu hình
 ****************************************************************************/

static constexpr int MAX_ESTIMATORS = 4;  // Match MAX_IMUS

/****************************************************************************
 * AttitudeModule - Tính tư thế từ dữ liệu IMU
 ****************************************************************************/

class AttitudeModule
{
public:
    AttitudeModule();
    ~AttitudeModule() = default;

    /**
     * @brief Khởi tạo estimator và subscribe IMU topics
     * @param imu_topics Mảng con trỏ tới IMU topic (từ ImuModule)
     * @param num_imus Số IMU đang hoạt động
     */
    void init(uorb::Topic<uorb::sensor_imu_s, 8>* imu_topics, int num_imus);

    /**
     * @brief Xử lý một chu kỳ cập nhật
     *
     * Được gọi từ main loop. Kiểm tra dữ liệu IMU mới, cập nhật estimator
     * và publish attitude.
     *
     * @return Số estimator đã cập nhật
     */
    int update();

    /**
     * @brief Lấy topic attitude cho một instance cụ thể
     */
    uorb::Topic<uorb::vehicle_attitude_s, 4>& get_topic(int instance)
    {
        return _att_topics[instance];
    }

    /**
     * @brief Lấy topic trạng thái estimator
     */
    uorb::Topic<uorb::estimator_status_s, 2>& get_status_topic(int instance)
    {
        return _status_topics[instance];
    }

    /**
     * @brief Lấy attitude mới nhất (tiện cho main)
     */
    bool get_attitude(int instance, uorb::vehicle_attitude_s& att);

    /**
     * @brief Kiểm tra instance hợp lệ
     */
    bool is_valid(int instance) const
    {
        return instance >= 0 && instance < _num_estimators;
    }

    /**
     * @brief Lấy số estimator
     */
    int num_estimators() const { return _num_estimators; }

private:
    // Subscription IMU (con trỏ tới topic của ImuModule)
    uorb::Subscription<uorb::sensor_imu_s, 8>* _imu_subs[MAX_ESTIMATORS];
    uorb::Topic<uorb::sensor_imu_s, 8>* _imu_topics[MAX_ESTIMATORS];

    // Các instance estimator
    attitude::AttitudeEstimatorQ _estimators[MAX_ESTIMATORS];
    uint64_t _last_imu_timestamp[MAX_ESTIMATORS];

    // Topic output
    uorb::Topic<uorb::vehicle_attitude_s, 4> _att_topics[MAX_ESTIMATORS];
    uorb::Topic<uorb::estimator_status_s, 2> _status_topics[MAX_ESTIMATORS];

    // Trạng thái
    int _num_estimators;
    uint32_t _update_count[MAX_ESTIMATORS];
};

} // namespace estimator
} // namespace modules
