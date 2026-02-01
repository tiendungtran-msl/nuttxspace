/****************************************************************************
 * apps/uav/lib/utils/dma_buffer.hpp
 *
 * DMA DOUBLE BUFFER - Buffer đôi cho zero-copy DMA transfers
 *
 * MỤC ĐÍCH:
 * - Cho phép DMA ghi vào một buffer trong khi CPU xử lý buffer kia
 * - Không có race condition - hoàn toàn deterministic
 * - Minimal latency cho sensor data acquisition
 *
 * THIẾT KẾ:
 * - Hai buffer được swap sau mỗi DMA complete
 * - Semaphore để thông báo buffer ready
 * - Timestamp được capture ngay khi DMA complete
 * - Aligned cho DMA requirements (32-byte alignment)
 *
 * TIMING DIAGRAM:
 *   DMA Fill Buffer 0    DMA Fill Buffer 1    DMA Fill Buffer 0
 *   |<---- 125 µs --->|  |<---- 125 µs --->|  |<---- 125 µs --->|
 *                    CPU Process B0        CPU Process B1
 *                     |<--- ~50 µs --->|    |<--- ~50 µs --->|
 *
 * SỬ DỤNG:
 *   DmaDoubleBuffer<ImuRawData, 4> imu_buffer;  // 4 IMUs per transfer
 *
 *   // In DMA complete ISR:
 *   imu_buffer.dma_complete_callback();
 *
 *   // In processing task:
 *   auto* data = imu_buffer.get_ready_buffer();
 *   process(data);
 *   imu_buffer.release_buffer();
 *
 ****************************************************************************/

#ifndef __UAV_LIB_UTILS_DMA_BUFFER_HPP
#define __UAV_LIB_UTILS_DMA_BUFFER_HPP

#include <stdint.h>
#include <string.h>
#include <semaphore.h>
#include <nuttx/semaphore.h>

namespace uav {
namespace utils {

/****************************************************************************
 * DmaDoubleBuffer - Zero-copy double buffer for DMA
 *
 * @tparam T    Element type (e.g., raw sensor data struct)
 * @tparam COUNT Number of elements per buffer
 ****************************************************************************/

template<typename T, uint32_t COUNT>
class DmaDoubleBuffer {
public:
    DmaDoubleBuffer() : m_active(0), m_timestamp(0), m_overruns(0), m_transfers(0) {
        memset(m_buffer, 0, sizeof(m_buffer));
        nxsem_init(&m_ready_sem, 0, 0);
        nxsem_set_protocol(&m_ready_sem, SEM_PRIO_NONE);
    }

    ~DmaDoubleBuffer() {
        nxsem_destroy(&m_ready_sem);
    }

    /* Delete copy/move */
    DmaDoubleBuffer(const DmaDoubleBuffer&) = delete;
    DmaDoubleBuffer& operator=(const DmaDoubleBuffer&) = delete;

    /**
     * @brief Lấy pointer đến buffer mà DMA đang/sẽ ghi vào
     *
     * Trả về buffer đang active - DMA sẽ ghi vào đây.
     *
     * @return Pointer đến active buffer
     */
    T* get_dma_buffer() {
        return m_buffer[m_active];
    }

    /**
     * @brief Lấy kích thước buffer cho DMA
     * @return Số bytes
     */
    constexpr size_t get_dma_buffer_size() const {
        return sizeof(T) * COUNT;
    }

    /**
     * @brief Callback khi DMA transfer hoàn thành
     *
     * GỌI TỪ ISR CONTEXT!
     * - Swap buffers
     * - Capture timestamp
     * - Signal semaphore
     */
    void dma_complete_callback() {
        /* Capture timestamp ngay lập tức */
        m_timestamp = get_current_time_us();

        /* Swap buffers atomically */
        m_active ^= 1;

        /* Increment transfer counter */
        m_transfers++;

        /* Signal processing task */
        nxsem_post(&m_ready_sem);
    }

    /**
     * @brief Đợi buffer ready (blocking)
     *
     * Task sẽ block cho đến khi có buffer mới.
     *
     * @return 0 nếu thành công, -errno nếu lỗi
     */
    int wait_ready() {
        return nxsem_wait(&m_ready_sem);
    }

    /**
     * @brief Đợi buffer ready với timeout
     *
     * @param timeout_us Timeout tính bằng microseconds
     * @return 0 nếu OK, -ETIMEDOUT nếu timeout
     */
    int wait_ready_timeout(uint32_t timeout_us) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);

        ts.tv_nsec += (timeout_us % 1000000) * 1000;
        ts.tv_sec += timeout_us / 1000000;
        if (ts.tv_nsec >= 1000000000) {
            ts.tv_nsec -= 1000000000;
            ts.tv_sec++;
        }

        return nxsem_timedwait(&m_ready_sem, &ts);
    }

    /**
     * @brief Lấy buffer đã sẵn sàng để xử lý
     *
     * Trả về buffer KHÔNG phải active (buffer vừa được DMA fill xong).
     *
     * @return Pointer đến ready buffer
     */
    const T* get_ready_buffer() const {
        return m_buffer[m_active ^ 1];  /* Opposite of active */
    }

    /**
     * @brief Lấy timestamp của lần DMA complete gần nhất
     * @return Timestamp tính bằng microseconds
     */
    uint64_t get_timestamp() const {
        return m_timestamp;
    }

    /**
     * @brief Lấy số lần overrun (buffer chưa được xử lý đã bị overwrite)
     * @return Số overruns
     */
    uint32_t get_overruns() const {
        return m_overruns;
    }

    /**
     * @brief Lấy tổng số transfers đã thực hiện
     * @return Số transfers
     */
    uint32_t get_transfer_count() const {
        return m_transfers;
    }

    /**
     * @brief Số elements trong mỗi buffer
     */
    constexpr uint32_t count() const {
        return COUNT;
    }

    /**
     * @brief Reset statistics
     */
    void reset_stats() {
        m_overruns = 0;
        m_transfers = 0;
    }

private:
    /* Helper để lấy thời gian hiện tại (implemented elsewhere) */
    static uint64_t get_current_time_us();

    /* Hai buffers - aligned cho DMA */
    alignas(32) T m_buffer[2][COUNT];

    /* Index của buffer đang được DMA ghi */
    volatile uint8_t m_active;

    /* Timestamp của lần transfer gần nhất */
    volatile uint64_t m_timestamp;

    /* Statistics */
    volatile uint32_t m_overruns;
    volatile uint32_t m_transfers;

    /* Semaphore để thông báo buffer ready */
    sem_t m_ready_sem;
};

/****************************************************************************
 * DmaTripleBuffer - Triple buffer cho pipeline processing
 *
 * Khi processing time > transfer time, dùng triple buffer để tránh stall.
 * - Buffer 0: DMA đang ghi
 * - Buffer 1: Đang được xử lý
 * - Buffer 2: Đã xử lý xong, chờ consumer
 *
 * Phức tạp hơn double buffer, chỉ dùng khi cần thiết.
 ****************************************************************************/

/* TODO: Implement nếu cần - hiện tại double buffer đủ */

} /* namespace utils */
} /* namespace uav */

/****************************************************************************
 * Implementation of time function
 ****************************************************************************/

#include <nuttx/clock.h>
#include <time.h>

template<typename T, uint32_t COUNT>
uint64_t uav::utils::DmaDoubleBuffer<T, COUNT>::get_current_time_us()
{
    struct timespec ts;
    clock_systime_timespec(&ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

#endif /* __UAV_LIB_UTILS_DMA_BUFFER_HPP */
