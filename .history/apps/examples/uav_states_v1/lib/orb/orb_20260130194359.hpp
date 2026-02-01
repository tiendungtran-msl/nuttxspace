/****************************************************************************
 * apps/examples/uav_states_v1/lib/orb/orb.hpp
 *
 * Lightweight Object Request Broker - inspired by PX4's uORB
 * Real-time pub-sub messaging for NuttX
 * 
 * Features:
 * - Simple ring buffer with spinlock protection
 * - Timestamped messages with freshness tracking
 * - Multi-instance support (e.g., 4 IMUs)
 * 
 * Note: Uses NuttX spinlock instead of C++ std::atomic
 ****************************************************************************/

#pragma once

#include <nuttx/config.h>
#include <nuttx/spinlock.h>
#include <nuttx/irq.h>
#include <cstdint>
#include <cstring>

namespace orb
{

//=============================================================================
// Timestamps
//=============================================================================

using timestamp_t = uint64_t;  // Microseconds since boot

/**
 * @brief Get current time in microseconds
 */
timestamp_t hrt_absolute_time();

//=============================================================================
// Message Base
//=============================================================================

/**
 * @brief Base structure for all ORB messages
 */
struct MessageBase
{
    timestamp_t timestamp;      ///< Sample timestamp (us)
    uint8_t     instance_id;    ///< Sensor instance (0-3 for 4x IMU)
    
    MessageBase() : timestamp(0), instance_id(0) {}
};

//=============================================================================
// Ring Buffer (Simple with spinlock)
//=============================================================================

template <typename T, size_t SIZE = 8>
class RingBuffer
{
    static_assert((SIZE & (SIZE - 1)) == 0, "SIZE must be power of 2");
    
public:
    RingBuffer() : _head(0), _tail(0)
    {
        spin_lock_init(&_lock);
    }
    
    /**
     * @brief Push message (producer side)
     * @return true always (overwrites oldest if full)
     */
    bool push(const T& msg)
    {
        irqstate_t flags = spin_lock_irqsave(&_lock);
        
        size_t next = (_head + 1) & (SIZE - 1);
        
        if (next == _tail) {
            // Buffer full - overwrite oldest
            _tail = (_tail + 1) & (SIZE - 1);
        }
        
        _buffer[_head] = msg;
        _head = next;
        
        spin_unlock_irqrestore(&_lock, flags);
        return true;
    }
    
    /**
     * @brief Pop message (consumer side)
     * @return true if data available
     */
    bool pop(T& msg)
    {
        irqstate_t flags = spin_lock_irqsave(&_lock);
        
        if (_tail == _head) {
            spin_unlock_irqrestore(&_lock, flags);
            return false;  // Empty
        }
        
        msg = _buffer[_tail];
        _tail = (_tail + 1) & (SIZE - 1);
        
        spin_unlock_irqrestore(&_lock, flags);
        return true;
    }
    
    /**
     * @brief Peek latest without consuming
     * @return true if data available
     */
    bool peek_latest(T& msg) const
    {
        irqstate_t flags = spin_lock_irqsave(const_cast<spinlock_t*>(&_lock));
        
        if (_head == _tail) {
            spin_unlock_irqrestore(const_cast<spinlock_t*>(&_lock), flags);
            return false;  // Empty
        }
        
        // Get most recent (head-1)
        size_t latest = (_head - 1 + SIZE) & (SIZE - 1);
        msg = _buffer[latest];
        
        spin_unlock_irqrestore(const_cast<spinlock_t*>(&_lock), flags);
        return true;
    }
    
    /**
     * @brief Check if new data available
     */
    bool updated() const
    {
        return _head != _tail;
    }
    
    /**
     * @brief Clear buffer
     */
    void clear()
    {
        irqstate_t flags = spin_lock_irqsave(&_lock);
        _tail = _head;
        spin_unlock_irqrestore(&_lock, flags);
    }
    
    /**
     * @brief Number of messages in buffer
     */
    size_t count() const
    {
        return (_head - _tail + SIZE) & (SIZE - 1);
    }

private:
    T _buffer[SIZE];
    volatile size_t _head;
    volatile size_t _tail;
    mutable spinlock_t _lock;
};

//=============================================================================
// Topic (Single Publisher, Multiple Subscribers)
//=============================================================================

template <typename T, size_t QUEUE_SIZE = 8>
class Topic
{
public:
    Topic() : _generation(0), _published(false)
    {
        spin_lock_init(&_lock);
    }
    
    /**
     * @brief Publish new message
     */
    void publish(const T& msg)
    {
        irqstate_t flags = spin_lock_irqsave(&_lock);
        _latest = msg;
        _generation++;
        _published = true;
        spin_unlock_irqrestore(&_lock, flags);
        
        // Also push to queue for subscribers that poll
        _queue.push(msg);
    }
    
    /**
     * @brief Get latest message (zero-copy read)
     * @return true if message available
     */
    bool get_latest(T& msg) const
    {
        if (!_published) {
            return false;
        }
        
        irqstate_t flags = spin_lock_irqsave(const_cast<spinlock_t*>(&_lock));
        msg = _latest;
        spin_unlock_irqrestore(const_cast<spinlock_t*>(&_lock), flags);
        return true;
    }
    
    /**
     * @brief Check if new data since given generation
     */
    bool updated_since(uint32_t gen) const
    {
        return _generation > gen;
    }
    
    /**
     * @brief Get current generation counter
     */
    uint32_t generation() const { return _generation; }
    
    /**
     * @brief Check if any data published
     */
    bool published() const { return _published; }
    
    /**
     * @brief Copy latest data (convenience method)
     * @return true if data available
     */
    bool copy(T& msg) const
    {
        return get_latest(msg);
    }
    
    /**
     * @brief Access queue for pop-style consumers
     */
    RingBuffer<T, QUEUE_SIZE>& queue() { return _queue; }

private:
    T _latest;
    volatile uint32_t _generation;
    volatile bool _published;
    mutable spinlock_t _lock;
    RingBuffer<T, QUEUE_SIZE> _queue;
};

//=============================================================================
// Subscription (Subscriber Handle)
//=============================================================================

template <typename T, size_t QUEUE_SIZE = 8>
class Subscription
{
public:
    Subscription() : _topic(nullptr), _last_gen(0) {}
    
    explicit Subscription(Topic<T, QUEUE_SIZE>& topic) 
        : _topic(&topic), _last_gen(0) {}
    
    void subscribe(Topic<T, QUEUE_SIZE>& topic)
    {
        _topic = &topic;
        _last_gen = 0;
    }
    
    /**
     * @brief Check if new data available
     */
    bool updated() const
    {
        return _topic && _topic->updated_since(_last_gen);
    }
    
    /**
     * @brief Copy latest data and update generation
     * @return true if new data was copied
     */
    bool copy(T& msg)
    {
        if (!_topic) {
            return false;
        }
        
        if (!_topic->get_latest(msg)) {
            return false;
        }
        
        _last_gen = _topic->generation();
        return true;
    }
    
    /**
     * @brief Get data only if updated since last call
     */
    bool update(T& msg)
    {
        if (!updated()) {
            return false;
        }
        return copy(msg);
    }

private:
    Topic<T, QUEUE_SIZE>* _topic;
    uint32_t _last_gen;
};

//=============================================================================
// Multi-Instance Topic Array
//=============================================================================

constexpr size_t ORB_MULTI_MAX_INSTANCES = 4;

template <typename T, size_t QUEUE_SIZE = 8>
class MultiTopic
{
public:
    Topic<T, QUEUE_SIZE>& operator[](size_t idx)
    {
        return _instances[idx < ORB_MULTI_MAX_INSTANCES ? idx : 0];
    }
    
    const Topic<T, QUEUE_SIZE>& operator[](size_t idx) const
    {
        return _instances[idx < ORB_MULTI_MAX_INSTANCES ? idx : 0];
    }
    
    static constexpr size_t max_instances() { return ORB_MULTI_MAX_INSTANCES; }

private:
    Topic<T, QUEUE_SIZE> _instances[ORB_MULTI_MAX_INSTANCES];
};

}  // namespace orb
