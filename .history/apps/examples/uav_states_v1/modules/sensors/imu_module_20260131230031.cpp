/****************************************************************************
 * apps/examples/uav_states_v1/modules/sensors/imu_module.cpp
 *
 * Triển khai module IMU
 *
 * Module này:
 * 1. Khởi tạo tuần tự tối đa 4 ICM-42688-P.
 * 2. Hiệu chuẩn bias gyro khi khởi động (giữ UAV đứng yên).
 * 3. Chạy thread producer đọc cảm biến và publish lên uORB.
 *
 * Thread producer là phần DUY NHẤT truy cập SPI cho IMU,
 * tránh tranh chấp bus khi có nhiều thread.
 *
 ****************************************************************************/

#include "imu_module.hpp"
#include "../../platforms/nuttx/hrt/hrt.h"

#include <nuttx/config.h>
#include <syslog.h>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <unistd.h>

namespace modules
{
namespace sensors
{

/****************************************************************************
 * Constructor / Destructor
 ****************************************************************************/

ImuModule::ImuModule()
    : _num_active(0)
    , _running(false)
    , _thread_started(false)
{
    memset(_sensors, 0, sizeof(_sensors));
    memset(_sensor_ok, 0, sizeof(_sensor_ok));
}

ImuModule::~ImuModule()
{
    stop();
    for (int i = 0; i < MAX_IMUS; i++) {
        if (_sensors[i]) {
            delete _sensors[i];
            _sensors[i] = nullptr;
        }
    }
}

/****************************************************************************
 * init - Khởi tạo tất cả IMU khả dụng
 *
 * Khởi tạo tuần tự có delay để tránh tranh chấp SPI.
 * Mỗi cảm biến có một chip select riêng.
 ****************************************************************************/

int ImuModule::init()
{
    // Device ID cho từng IMU (định nghĩa trong spi_config.h)
    static const uint32_t devids[MAX_IMUS] = {
        SPIDEV_IMU, SPIDEV_IMU1, SPIDEV_IMU2, SPIDEV_IMU3
    };

    syslog(LOG_INFO, "[sensors] Initializing IMU sensors...\n");
    _num_active = 0;

    for (int i = 0; i < MAX_IMUS; i++) {
        syslog(LOG_INFO, "[sensors]   IMU%d: ", i);

        // Tạo instance cảm biến
        _sensors[i] = new drivers::imu::ICM42688P(NUTTX_SPI_BUS_IMU, devids[i]);
        if (!_sensors[i]) {
            syslog(LOG_ERR, "allocation failed\n");
            continue;
        }

        // Khởi tạo cảm biến
        int ret = _sensors[i]->initialize();
        if (ret == 0) {
            _sensor_ok[i] = true;
            _num_active++;
            syslog(LOG_INFO, "OK\n");
        } else {
            syslog(LOG_ERR, "init failed (%d)\n", ret);
            delete _sensors[i];
            _sensors[i] = nullptr;
        }

        // Delay giữa các lần init để tránh tranh chấp bus
        usleep(10000);
    }

    syslog(LOG_INFO, "[sensors] Active sensors: %d/%d\n", _num_active, MAX_IMUS);

    // Khởi tạo uORB topics
    for (int i = 0; i < MAX_IMUS; i++) {
        _topics[i].init();
    }

    return _num_active;
}

/****************************************************************************
 * calibrate - Hiệu chuẩn bias gyro
 *
 * Thu thập mẫu khi UAV đứng yên và tính bias trung bình.
 * Đồng thời tính hệ số scale cho accel (đứng yên phải ~1g).
 ****************************************************************************/

bool ImuModule::calibrate()
{
    if (_num_active == 0) {
        return false;
    }

    syslog(LOG_INFO, "[sensors] Calibrating (keep still for %d ms)...\n",
           CALIBRATION_TIME_MS);

    // Chờ cảm biến ổn định
    usleep(500000);

    const int samples = (CALIBRATION_TIME_MS * SENSOR_RATE_HZ) / 1000;

    for (int i = 0; i < MAX_IMUS; i++) {
        if (!_sensor_ok[i]) continue;

        float gyro_sum[3] = {0, 0, 0};
        float accel_sum[3] = {0, 0, 0};
        int count = 0;

        syslog(LOG_INFO, "[sensors]   IMU%d: ", i);

        for (int s = 0; s < samples; s++) {
            drivers::imu::ICM42688P::Data data;
            if (_sensors[i]->read(data) == 0) {
                gyro_sum[0] += data.gyro[0];
                gyro_sum[1] += data.gyro[1];
                gyro_sum[2] += data.gyro[2];
                accel_sum[0] += data.accel[0];
                accel_sum[1] += data.accel[1];
                accel_sum[2] += data.accel[2];
                count++;
            }
            usleep(1000000 / SENSOR_RATE_HZ);
        }

        if (count < samples / 2) {
            syslog(LOG_ERR, "insufficient samples\n");
            _sensor_ok[i] = false;
            _num_active--;
            continue;
        }

        float inv = 1.0f / count;

        // Tính và set bias gyro
        float gyro_bias[3] = {
            gyro_sum[0] * inv,
            gyro_sum[1] * inv,
            gyro_sum[2] * inv
        };
        _sensors[i]->set_gyro_bias(gyro_bias);

        // Tính hệ số scale cho accel
        float ax = accel_sum[0] * inv;
        float ay = accel_sum[1] * inv;
        float az = accel_sum[2] * inv;
        float norm = sqrtf(ax*ax + ay*ay + az*az);
        float scale = (norm > 1.0f) ? (GRAVITY / norm) : 1.0f;
        _sensors[i]->set_accel_scale_correction(scale);

        syslog(LOG_INFO, "OK (bias: %.3f, %.3f, %.3f rad/s)\n",
               (double)gyro_bias[0], (double)gyro_bias[1], (double)gyro_bias[2]);
    }

    return _num_active > 0;
}

/****************************************************************************
 * start / stop - Quản lý thread
 ****************************************************************************/

int ImuModule::start()
{
    if (_thread_started) {
        return 0;
    }

    _running = true;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 4096);

    // Ưu tiên cao hơn cho thread đọc cảm biến (quan trọng về thời gian)
    struct sched_param param;
    param.sched_priority = SCHED_PRIORITY_DEFAULT + 10;
    pthread_attr_setschedparam(&attr, &param);

    int ret = pthread_create(&_thread, &attr, thread_entry, this);
    pthread_attr_destroy(&attr);

    if (ret != 0) {
        syslog(LOG_ERR, "[sensors] Failed to create thread: %d\n", ret);
        _running = false;
        return -ret;
    }

    _thread_started = true;
    syslog(LOG_INFO, "[sensors] Sensor thread started @ %d Hz\n", SENSOR_RATE_HZ);
    return 0;
}

void ImuModule::stop()
{
    if (!_thread_started) {
        return;
    }

    _running = false;
    pthread_join(_thread, nullptr);
    _thread_started = false;
    syslog(LOG_INFO, "[sensors] Sensor thread stopped\n");
}

/****************************************************************************
 * Thread - Vòng lặp đọc cảm biến
 *
 * Chạy ở SENSOR_RATE_HZ, đọc tất cả cảm biến và publish lên topic.
 * Dùng hrt_absolute_time() để canh thời gian chính xác.
 ****************************************************************************/

void* ImuModule::thread_entry(void* arg)
{
    ImuModule* self = static_cast<ImuModule*>(arg);
    self->run();
    return nullptr;
}

void ImuModule::run()
{
    const uint32_t loop_period_us = 1000000 / SENSOR_RATE_HZ;
    uint64_t next_time = hrt_absolute_time();

    while (_running) {
        // Đọc tất cả cảm biến theo thứ tự
        for (int i = 0; i < MAX_IMUS; i++) {
            if (!_sensor_ok[i]) {
                continue;
            }

            drivers::imu::ICM42688P::Data data;
            if (_sensors[i]->read(data) == 0) {
                // Đóng gói vào message uORB
                uorb::sensor_imu_s msg{};
                msg.timestamp_us = data.timestamp_us;
                msg.accel[0] = data.accel[0];
                msg.accel[1] = data.accel[1];
                msg.accel[2] = data.accel[2];
                msg.gyro[0] = data.gyro[0];
                msg.gyro[1] = data.gyro[1];
                msg.gyro[2] = data.gyro[2];
                msg.temperature = data.temperature;
                msg.instance = static_cast<uint8_t>(i);

                // Publish lên topic
                _topics[i].publish(msg);
            }
        }

        // Điều khiển thời gian chính xác
        next_time += loop_period_us;
        int64_t sleep_us = (int64_t)next_time - (int64_t)hrt_absolute_time();
        if (sleep_us > 0) {
            usleep(sleep_us);
        } else {
            // Quá thời gian - reset mốc thời gian
            next_time = hrt_absolute_time();
        }
    }
}

} // namespace sensors
} // namespace modules
