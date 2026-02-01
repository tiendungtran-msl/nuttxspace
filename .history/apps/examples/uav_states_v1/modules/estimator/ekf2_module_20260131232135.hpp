/****************************************************************************
 * apps/examples/uav_states_v1/modules/estimator/ekf2_module.hpp
 *
 * Module EKF2 - Extended Kalman Filter cho state estimation
 *
 * MỤC ĐÍCH:
 * - Nhận dữ liệu từ các cảm biến (IMU, GPS, Mag, Baro).
 * - Fusion dùng EKF2 để ước lượng đầy đủ trạng thái.
 * - Publish position, velocity, attitude đã lọc.
 *
 * KIẾN TRÚC:
 * - Subscribe: sensor_imu, sensor_gps, sensor_mag, sensor_baro
 * - Publish: vehicle_attitude, vehicle_local_position, ekf2_status
 *
 * LUỒNG DỮ LIỆU:
 *   IMU ---\
 *   GPS --->  EKF2Module --> vehicle_local_position
 *   Mag ---/               --> vehicle_attitude
 *   Baro --/               --> ekf2_status
 *
 ****************************************************************************/

#pragma once

#include <cstdint>

#include "../../uorb/uorb.hpp"
#include "../../uorb/topics.hpp"
#include "../../lib/ekf2/ekf2.hpp"

namespace modules
{
namespace estimator
{

/****************************************************************************
 * Cấu hình
 ****************************************************************************/

static constexpr int EKF2_QUEUE_DEPTH = 8;

/****************************************************************************
 * Ekf2Module - Full state estimator
 ****************************************************************************/

class Ekf2Module
{
public:
    Ekf2Module();
    ~Ekf2Module() = default;

    /**
     * @brief Khởi tạo EKF2 và subscribe các topics
     * @param imu_topic Con trỏ tới IMU topic (primary IMU)
     */
    void init(uorb::Topic<uorb::sensor_imu_s, 8>* imu_topic);

    /**
     * @brief Xử lý một chu kỳ cập nhật
     *
     * Gọi từ main loop. Kiểm tra dữ liệu mới từ các cảm biến,
     * chạy EKF2 update và publish kết quả.
     *
     * @return true nếu có update
     */
    bool update();

    //=========================================================================
    // Sensor input - gọi khi có dữ liệu mới từ driver
    //=========================================================================

    /**
     * @brief Cung cấp dữ liệu GPS
     * Gọi khi GPS driver có sample mới.
     */
    void setGpsData(const uorb::sensor_gps_s& gps);

    /**
     * @brief Cung cấp dữ liệu Magnetometer
     */
    void setMagData(const uorb::sensor_mag_s& mag);

    /**
     * @brief Cung cấp dữ liệu Barometer
     */
    void setBaroData(const uorb::sensor_baro_s& baro);

    //=========================================================================
    // Output topics
    //=========================================================================

    uorb::Topic<uorb::vehicle_local_position_s, 4>& get_local_pos_topic()
    {
        return _local_pos_topic;
    }

    uorb::Topic<uorb::vehicle_attitude_s, 4>& get_attitude_topic()
    {
        return _attitude_topic;
    }

    uorb::Topic<uorb::ekf2_status_s, 2>& get_status_topic()
    {
        return _status_topic;
    }

    //=========================================================================
    // Getters
    //=========================================================================

    /**
     * @brief Lấy attitude mới nhất
     */
    bool get_attitude(uorb::vehicle_attitude_s& att);

    /**
     * @brief Lấy local position mới nhất
     */
    bool get_local_position(uorb::vehicle_local_position_s& pos);

    /**
     * @brief Lấy EKF2 state trực tiếp
     */
    const ekf2::StateSample& get_state() const { return _ekf.getState(); }

    /**
     * @brief Kiểm tra filter đã sẵn sàng
     */
    bool is_initialized() const { return _filter_init; }

private:
    // EKF2 filter instance
    ekf2::Ekf2 _ekf;

    // Input topic pointer (IMU từ ImuModule)
    uorb::Topic<uorb::sensor_imu_s, 8>* _imu_topic;

    // Output topics
    uorb::Topic<uorb::vehicle_local_position_s, 4> _local_pos_topic;
    uorb::Topic<uorb::vehicle_attitude_s, 4> _attitude_topic;
    uorb::Topic<uorb::ekf2_status_s, 2> _status_topic;

    // Subscription handle (initialized in init())
    uorb::Subscription<uorb::sensor_imu_s, 8>* _imu_sub;

    // Last published data
    uorb::vehicle_local_position_s _last_local_pos;
    uorb::vehicle_attitude_s _last_attitude;

    // Filter initialized flag
    bool _filter_init;

    // Update counter
    uint32_t _update_count;

    /**
     * @brief Publish attitude từ EKF state
     */
    void publishAttitude(uint64_t timestamp);

    /**
     * @brief Publish local position từ EKF state
     */
    void publishLocalPosition(uint64_t timestamp);

    /**
     * @brief Publish EKF2 status
     */
    void publishStatus(uint64_t timestamp);
};

} // namespace estimator
} // namespace modules
