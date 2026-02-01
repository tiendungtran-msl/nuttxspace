/****************************************************************************
 * apps/examples/uav_states_v1/uorb/uorb.hpp
 *
 * Hệ thống Pub/Sub kiểu uORB (nhẹ)
 *
 * MỤC ĐÍCH:
 * - Tách I/O cảm biến khỏi xử lý dữ liệu (thiết kế module).
 * - Mỗi module (sensor, estimator, logger) chạy độc lập.
 * - Broker trung tâm đảm bảo trao đổi dữ liệu an toàn đa luồng.
 * - Lộ trình nâng cấp: có thể chuyển sang uORB đa tiến trình về sau.
 *
 * KIẾN TRÚC:
 * - Publisher: ghi dữ liệu vào topic (ring buffer + số thứ tự).
 * - Subscriber: đọc mẫu mới nhất, bỏ qua dữ liệu trùng qua seq.
 * - Topic: ring buffer có kiểu, bảo vệ bằng mutex.
 *
 * CÁCH DÙNG:
 *   // Tạo topic
 *   static uorb::Topic<ImuSample, 8> g_imu_topic;
 *   
 *   // Publisher
 *   ImuSample sample = { ... };
 *   g_imu_topic.publish(sample);
 *   
 *   // Subscriber
 *   uorb::Subscription<ImuSample> sub(g_imu_topic);
 *   if (sub.updated()) {
 *       ImuSample data;
 *       sub.copy(data);
 *   }
 *
 ****************************************************************************/

#pragma once

#include <pthread.h>
#include <cstdint>
#include <cstring>

namespace uorb
{

/****************************************************************************
 * Topic Template - Ring buffer có mutex bảo vệ
 *
 * DEPTH: Số phần tử trong ring buffer. Lớn hơn = lưu lịch sử nhiều hơn.
 *        Với cảm biến 100Hz, DEPTH=8 tương đương ~80ms dữ liệu.
 ****************************************************************************/

template <typename T, uint8_t DEPTH = 8>
class Topic
{
public:
    Topic() : _seq(0), _write_idx(0), _initialized(false)
    {
        memset(_buffer, 0, sizeof(_buffer));
    }

    ~Topic()
    {
        if (_initialized) {
            pthread_mutex_destroy(&_lock);
        }
    }

    /**
     * @brief Khởi tạo topic (bắt buộc trước khi dùng)
     */
    void init()
    {
        if (!_initialized) {
            pthread_mutex_init(&_lock, nullptr);
            _seq = 0;
            _write_idx = 0;
            _initialized = true;
        }
    }

    /**
     * @brief Publish mẫu mới vào topic
     * @param sample Dữ liệu cần publish
     *
     * Được gọi bởi producer (ví dụ thread cảm biến).
     * An toàn đa luồng: dùng mutex.
     */
    void publish(const T& sample)
    {
        if (!_initialized) {
            init();
        }

        pthread_mutex_lock(&_lock);
        _buffer[_write_idx] = sample;
        _write_idx = (_write_idx + 1) % DEPTH;
        _seq++;
        pthread_mutex_unlock(&_lock);
    }

    /**
     * @brief Copy mẫu mới nhất nếu có cập nhật từ lần trước
     * @param last_seq Bộ đếm seq của caller (được cập nhật khi copy)
     * @param out Bộ đệm output
     * @return true nếu có dữ liệu mới, false nếu không cập nhật
     *
     * Được gọi bởi consumer (ví dụ thread estimator).
     * An toàn đa luồng: dùng mutex.
     */
    bool copy_if_updated(uint32_t& last_seq, T& out)
    {
        if (!_initialized) {
            return false;
        }

        pthread_mutex_lock(&_lock);
        if (_seq == last_seq) {
            pthread_mutex_unlock(&_lock);
            return false;
        }

        // Lấy phần tử mới nhất (trước con trỏ ghi)
        uint8_t latest_idx = (_write_idx == 0) ? (DEPTH - 1) : (_write_idx - 1);
        out = _buffer[latest_idx];
        last_seq = _seq;
        pthread_mutex_unlock(&_lock);
        return true;
    }

    /**
     * @brief Copy mẫu mới nhất không điều kiện
     * @param out Bộ đệm output
     * @return true nếu có dữ liệu
     */
    bool copy(T& out)
    {
        if (!_initialized || _seq == 0) {
            return false;
        }

        pthread_mutex_lock(&_lock);
        uint8_t latest_idx = (_write_idx == 0) ? (DEPTH - 1) : (_write_idx - 1);
        out = _buffer[latest_idx];
        pthread_mutex_unlock(&_lock);
        return true;
    }

    /**
     * @brief Lấy số thứ tự hiện tại
     */
    uint32_t sequence() const { return _seq; }

    /**
     * @brief Kiểm tra topic đã có dữ liệu hay chưa
     */
    bool valid() const { return _initialized && _seq > 0; }

private:
    pthread_mutex_t _lock;
    T _buffer[DEPTH];
    uint32_t _seq;
    uint8_t _write_idx;
    bool _initialized;
};

/****************************************************************************
 * Subscription - Wrapper cho consumer đọc dữ liệu topic
 *
 * Theo dõi seq để phát hiện cập nhật.
 * Giao diện đơn giản cho consumer.
 ****************************************************************************/

template <typename T, uint8_t DEPTH = 8>
class Subscription
{
public:
    Subscription(Topic<T, DEPTH>& topic) : _topic(topic), _last_seq(0) {}

    /**
     * @brief Kiểm tra có dữ liệu mới không
     */
    bool updated() const
    {
        return _topic.sequence() != _last_seq;
    }

    /**
     * @brief Copy dữ liệu mới nhất nếu có
     * @param out Bộ đệm output
     * @return true nếu copy được dữ liệu mới
     */
    bool copy(T& out)
    {
        return _topic.copy_if_updated(_last_seq, out);
    }

    /**
     * @brief Ép copy dữ liệu mới nhất (dù đã đọc rồi)
     */
    bool copy_force(T& out)
    {
        return _topic.copy(out);
    }

private:
    Topic<T, DEPTH>& _topic;
    uint32_t _last_seq;
};

} // namespace uorb
