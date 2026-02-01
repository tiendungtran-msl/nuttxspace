/****************************************************************************
 * apps/examples/uav_states_v1/uorb/uorb.hpp
 *
 * Lightweight uORB-like Pub/Sub System
 *
 * PURPOSE:
 * - Decouples sensor I/O from data processing (modular design).
 * - Each module (sensor, estimator, logger) runs independently.
 * - Central topic broker handles data exchange with thread-safety.
 * - Migration path: can later become true multi-process uORB.
 *
 * ARCHITECTURE:
 * - Publisher: writes new data to topic (ring buffer + sequence number).
 * - Subscriber: reads latest data, skipping duplicates via seq tracking.
 * - Topics: typed ring buffers protected by mutex.
 *
 * USAGE:
 *   // Define topic
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
 * Topic Template - Ring buffer with mutex protection
 *
 * DEPTH: Number of entries in ring buffer. Larger = more history.
 *        For 100Hz sensor, DEPTH=8 gives ~80ms history.
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
     * @brief Initialize topic (must call before use)
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
     * @brief Publish new sample to topic
     * @param sample Data to publish
     *
     * Called by producer (e.g., sensor thread).
     * Thread-safe: uses mutex.
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
     * @brief Copy latest sample if updated since last call
     * @param last_seq Caller's sequence tracker (updated on copy)
     * @param out Output buffer
     * @return true if new data copied, false if no update
     *
     * Called by consumer (e.g., estimator thread).
     * Thread-safe: uses mutex.
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

        // Get latest entry (one before write pointer)
        uint8_t latest_idx = (_write_idx == 0) ? (DEPTH - 1) : (_write_idx - 1);
        out = _buffer[latest_idx];
        last_seq = _seq;
        pthread_mutex_unlock(&_lock);
        return true;
    }

    /**
     * @brief Copy latest sample unconditionally
     * @param out Output buffer
     * @return true if data available
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
     * @brief Get current sequence number
     */
    uint32_t sequence() const { return _seq; }

    /**
     * @brief Check if topic has data
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
 * Subscription - Wrapper for consuming topic data
 *
 * Tracks sequence number to detect updates.
 * Simpler interface for consumers.
 ****************************************************************************/

template <typename T, uint8_t DEPTH = 8>
class Subscription
{
public:
    Subscription(Topic<T, DEPTH>& topic) : _topic(topic), _last_seq(0) {}

    /**
     * @brief Check if new data is available
     */
    bool updated() const
    {
        return _topic.sequence() != _last_seq;
    }

    /**
     * @brief Copy latest data if available
     * @param out Output buffer
     * @return true if new data copied
     */
    bool copy(T& out)
    {
        return _topic.copy_if_updated(_last_seq, out);
    }

    /**
     * @brief Force copy latest data (even if already read)
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
