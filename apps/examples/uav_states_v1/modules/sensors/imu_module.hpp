/****************************************************************************
 * apps/examples/uav_states_v1/modules/sensors/imu_module.hpp
 *
 * Module IMU - Thu thập dữ liệu cảm biến
 *
 * MỤC ĐÍCH:
 * - Quản lý nhiều cảm biến ICM-42688-P.
 * - Đọc dữ liệu theo tần số cố định (PRODUCER trong pub/sub).
 * - Publish dữ liệu thô (đã bù bias) lên uORB topics.
 * - Xử lý khởi tạo, hiệu chuẩn, theo dõi sức khỏe.
 *
 * KIẾN TRÚC:
 * - Một thread sở hữu toàn bộ SPI -> tránh tranh chấp bus.
 * - Publish lên topic sensor_imu (mỗi sensor một topic).
 * - Consumer (estimator) subscribe độc lập.
 *
 * MỞ RỘNG:
 * - Thêm magnetometer (BMM150).
 * - Thêm barometer (MS5611).
 * - Theo dõi sức khỏe và voting cảm biến.
 *
 ****************************************************************************/

#pragma once

#include <pthread.h>
#include <cstdint>

#include "../../uorb/uorb.hpp"
#include "../../uorb/topics.hpp"
#include "../../drivers/imu/icm42688p/icm42688p.hpp"
#include "../../platforms/boards/spi_config.h"

namespace modules
{
namespace sensors
{

/****************************************************************************
 * Cấu hình
 ****************************************************************************/

static constexpr int MAX_IMUS = 4;
static constexpr int SENSOR_RATE_HZ = 100;
static constexpr int CALIBRATION_TIME_MS = 2000;
static constexpr float GRAVITY = 9.80665f;

/****************************************************************************
 * ImuModule - Quản lý IMU và publish dữ liệu
 ****************************************************************************/

class ImuModule
{
public:
    ImuModule();
    ~ImuModule();

    /**
     * @brief Khởi tạo tất cả IMU khả dụng
     * @return Số cảm biến khởi tạo thành công
     */
    int init();

    /**
     * @brief Hiệu chuẩn bias gyro (giữ UAV đứng yên khi hiệu chuẩn)
     * @return true nếu có ít nhất một cảm biến hiệu chuẩn thành công
     */
    bool calibrate();

    /**
     * @brief Bắt đầu thread đọc cảm biến (producer)
     * @return 0 nếu thành công, âm nếu lỗi
     */
    int start();

    /**
     * @brief Dừng thread đọc cảm biến
     */
    void stop();

    /**
     * @brief Yêu cầu dừng (không blocking, dùng cho signal handler)
     */
    void request_stop() { _running = false; }

    /**
     * @brief Lấy topic của một IMU cụ thể
     */
    uorb::Topic<uorb::sensor_imu_s, 8>& get_topic(int instance)
    {
        return _topics[instance];
    }

    /**
     * @brief Kiểm tra cảm biến có hoạt động không
     */
    bool is_active(int instance) const
    {
        return instance >= 0 && instance < MAX_IMUS && _sensor_ok[instance];
    }

    /**
     * @brief Lấy số cảm biến đang hoạt động
     */
    int num_active() const { return _num_active; }

private:
    // Thread entry point
    static void* thread_entry(void* arg);
    void run();

    // Sensors
    drivers::imu::ICM42688P* _sensors[MAX_IMUS];
    bool _sensor_ok[MAX_IMUS];
    int _num_active;

    // Topics (one per IMU)
    uorb::Topic<uorb::sensor_imu_s, 8> _topics[MAX_IMUS];

    // Thread
    pthread_t _thread;
    volatile bool _running;
    bool _thread_started;
};

} // namespace sensors
} // namespace modules
