/****************************************************************************
 * apps/uav/lib/utils/ringbuf.hpp
 *
 * LOCK-FREE RING BUFFER - Buffer tuần hoàn không cần lock
 *
 * MỤC ĐÍCH:
 * - Truyền dữ liệu an toàn giữa producer và consumer
 * - Không cần mutex/lock - tối ưu cho realtime
 * - Zero-copy khi có thể
 * - Deterministic performance
 *
 * THIẾT KẾ:
 * - Single Producer Single Consumer (SPSC) pattern
 * - Head/Tail pointers với memory barriers
 * - Power-of-2 size để dùng bitmask thay modulo
 * - Cache-line aligned để tránh false sharing
 *
 * SỬ DỤNG:
 *   RingBuffer<sensor_imu_s, 16> imu_ring;
 *   
 *   // Producer (ISR/DMA callback)
 *   imu_ring.push(sample);
 *   
 *   // Consumer (processing task)
 *   sensor_imu_s samples[8];
 *   int count = imu_ring.pop_batch(samples, 8);
 *
 * LƯU Ý:
 * - Size phải là power of 2 (2, 4, 8, 16, 32, ...)
 * - Chỉ an toàn với 1 producer và 1 consumer
 * - Nếu cần multi-producer, dùng variant có mutex
 *
 ****************************************************************************/

#ifndef __UAV_LIB_UTILS_RINGBUF_HPP
#define __UAV_LIB_UTILS_RINGBUF_HPP

#include <stdint.h>
#include <string.h>
#include <nuttx/compiler.h>
#include <nuttx/arch.h>

/**
 * Memory barrier macro - portable across architectures
 * Đảm bảo thứ tự memory operations
 */
#ifndef UAV_MEMORY_BARRIER
#  if defined(CONFIG_ARCH_ARM) || defined(CONFIG_ARCH_ARMV7M) || defined(CONFIG_ARCH_ARMV8M)
#    define UAV_MEMORY_BARRIER() __asm__ __volatile__("dmb" ::: "memory")
#  else
#    define UAV_MEMORY_BARRIER() __asm__ __volatile__("" ::: "memory")
#  endif
#endif

namespace uav {
namespace utils {

/****************************************************************************
 * Compile-time check: size phải là power of 2
 ****************************************************************************/

template<uint32_t N>
struct IsPowerOf2 {
    static constexpr bool value = (N > 0) && ((N & (N - 1)) == 0);
};

/****************************************************************************
 * RingBuffer - Lock-free SPSC ring buffer
 *
 * @tparam T    Element type
 * @tparam SIZE Buffer size (must be power of 2)
 ****************************************************************************/

template<typename T, uint32_t SIZE>
class RingBuffer {
    static_assert(IsPowerOf2<SIZE>::value,
                  "Ring buffer size must be power of 2");

public:
    RingBuffer() : m_head(0), m_tail(0) {
        memset(m_buffer, 0, sizeof(m_buffer));
    }

    /**
     * @brief Kiểm tra buffer có rỗng không
     * @return true nếu rỗng
     */
    bool empty() const {
        return m_head == m_tail;
    }

    /**
     * @brief Kiểm tra buffer có đầy không
     * @return true nếu đầy
     */
    bool full() const {
        return available() >= SIZE;
    }

    /**
     * @brief Số phần tử đang có trong buffer
     * @return Số phần tử
     */
    uint32_t available() const {
        return m_head - m_tail;  /* Works with wrap-around */
    }

    /**
     * @brief Số slot còn trống
     * @return Số slot free
     */
    uint32_t free_space() const {
        return SIZE - available();
    }

    /**
     * @brief Capacity của buffer
     * @return SIZE
     */
    constexpr uint32_t capacity() const {
        return SIZE;
    }

    /**
     * @brief Push một element vào buffer
     *
     * @param item Element cần push
     * @return true nếu thành công, false nếu buffer đầy
     */
    bool push(const T& item) {
        if (full()) {
            return false;  /* Overflow */
        }

        uint32_t head = m_head;
        m_buffer[head & MASK] = item;

        /* Memory barrier trước khi publish head mới */
        __DMB();

        m_head = head + 1;
        return true;
    }

    /**
     * @brief Push và overwrite nếu đầy
     *
     * Dùng khi muốn luôn có data mới nhất, chấp nhận mất data cũ.
     *
     * @param item Element cần push
     * @return true nếu overwrite, false nếu không
     */
    bool push_overwrite(const T& item) {
        bool overwritten = false;

        if (full()) {
            /* Advance tail to make room */
            m_tail++;
            overwritten = true;
        }

        uint32_t head = m_head;
        m_buffer[head & MASK] = item;

        __DMB();
        m_head = head + 1;

        return overwritten;
    }

    /**
     * @brief Pop một element ra khỏi buffer
     *
     * @param item Output - element được pop ra
     * @return true nếu thành công, false nếu buffer rỗng
     */
    bool pop(T& item) {
        if (empty()) {
            return false;  /* Underflow */
        }

        uint32_t tail = m_tail;
        item = m_buffer[tail & MASK];

        __DMB();
        m_tail = tail + 1;

        return true;
    }

    /**
     * @brief Peek element tiếp theo mà không pop
     *
     * @param item Output - element được peek
     * @return true nếu có data, false nếu rỗng
     */
    bool peek(T& item) const {
        if (empty()) {
            return false;
        }

        item = m_buffer[m_tail & MASK];
        return true;
    }

    /**
     * @brief Pop nhiều elements cùng lúc
     *
     * Hiệu quả hơn pop() nhiều lần vì giảm overhead.
     *
     * @param items Output array
     * @param max_count Số element tối đa cần pop
     * @return Số element thực tế đã pop
     */
    uint32_t pop_batch(T* items, uint32_t max_count) {
        uint32_t avail = available();
        uint32_t count = (avail < max_count) ? avail : max_count;

        uint32_t tail = m_tail;
        for (uint32_t i = 0; i < count; i++) {
            items[i] = m_buffer[(tail + i) & MASK];
        }

        __DMB();
        m_tail = tail + count;

        return count;
    }

    /**
     * @brief Clear buffer
     */
    void clear() {
        m_tail = m_head;
    }

    /**
     * @brief Drain - pop tất cả elements
     *
     * @param items Output array (phải đủ lớn)
     * @return Số element đã drain
     */
    uint32_t drain(T* items) {
        return pop_batch(items, available());
    }

    /**
     * @brief Get element by index (không pop)
     *
     * @param index Index từ tail (0 = oldest)
     * @param item Output
     * @return true nếu index hợp lệ
     */
    bool get_at(uint32_t index, T& item) const {
        if (index >= available()) {
            return false;
        }

        item = m_buffer[(m_tail + index) & MASK];
        return true;
    }

    /**
     * @brief Get newest element (không pop)
     *
     * @param item Output
     * @return true nếu có data
     */
    bool get_newest(T& item) const {
        if (empty()) {
            return false;
        }

        item = m_buffer[(m_head - 1) & MASK];
        return true;
    }

private:
    static constexpr uint32_t MASK = SIZE - 1;

    /* Buffer storage - aligned cho cache efficiency */
    alignas(32) T m_buffer[SIZE];

    /* Head - nơi producer ghi (volatile cho visibility) */
    volatile uint32_t m_head;

    /* Tail - nơi consumer đọc (volatile cho visibility) */
    volatile uint32_t m_tail;
};

/****************************************************************************
 * TimestampedRingBuffer - Ring buffer với timestamp cho mỗi sample
 *
 * Useful cho sensor data cần biết thời điểm capture.
 ****************************************************************************/

template<typename T>
struct TimestampedSample {
    uint64_t timestamp_us;
    T data;
};

template<typename T, uint32_t SIZE>
using TimestampedRingBuffer = RingBuffer<TimestampedSample<T>, SIZE>;

/****************************************************************************
 * MultiWriterRingBuffer - Ring buffer cho multiple producers
 *
 * Dùng khi có nhiều sources ghi vào cùng buffer.
 * Chậm hơn SPSC version do cần mutex.
 ****************************************************************************/

/* TODO: Implement nếu cần - hiện tại dùng separate buffers cho mỗi IMU */

} /* namespace utils */
} /* namespace uav */

#endif /* __UAV_LIB_UTILS_RINGBUF_HPP */
