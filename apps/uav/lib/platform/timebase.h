/****************************************************************************
 * apps/uav/lib/platform/timebase.h
 *
 * MASTER TIMEBASE - Nhân thời gian trung tâm cho hệ thống UAV
 *
 * MỤC ĐÍCH:
 * - Cung cấp tick chính xác cho toàn hệ thống (master rate: 8 kHz)
 * - Phân phối tick cho các domain: FAST, MEDIUM, SLOW, ASYNC
 * - Đảm bảo deterministic timing cho realtime tasks
 * - Giảm thiểu jitter bằng hardware timer
 *
 * THIẾT KẾ:
 * - Hardware timer (TIM5 trên STM32H7) tạo interrupt mỗi 125 µs
 * - ISR chỉ set flags và post semaphores - không xử lý nặng
 * - Mỗi domain có semaphore riêng để synchronize tasks
 * - Timestamp chính xác microsecond qua HRT
 *
 * TIMING:
 * - Master: 8 kHz (125 µs period)
 * - FAST domain: 8 kHz (mỗi tick) - IMU DMA, gyro integration
 * - MEDIUM domain: 1 kHz (8 ticks) - EKF predict, sensor sync
 * - SLOW domain: 100 Hz (80 ticks) - Baro, Mag, GPS fusion
 * - ASYNC domain: Event-driven - External events, telemetry
 *
 * SỬ DỤNG:
 *   1. Gọi timebase_init() một lần khi khởi động
 *   2. Các task gọi timebase_wait_domain(domain) để sync với tick
 *   3. Gọi timebase_get_timestamp() để lấy thời gian chính xác
 *   4. Gọi timebase_deinit() khi shutdown
 *
 ****************************************************************************/

#ifndef __UAV_LIB_PLATFORM_TIMEBASE_H
#define __UAV_LIB_PLATFORM_TIMEBASE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Cấu hình Master Tick - Có thể override qua Kconfig */

#ifndef CONFIG_UAV_MASTER_TICK_HZ
#define CONFIG_UAV_MASTER_TICK_HZ       8000    /* 8 kHz master rate */
#endif

#ifndef CONFIG_UAV_MEDIUM_DIVIDER
#define CONFIG_UAV_MEDIUM_DIVIDER       8       /* 8 kHz / 8 = 1 kHz */
#endif

#ifndef CONFIG_UAV_SLOW_DIVIDER
#define CONFIG_UAV_SLOW_DIVIDER         80      /* 8 kHz / 80 = 100 Hz */
#endif

/* Derived constants */
#define TIMEBASE_MASTER_PERIOD_US       (1000000 / CONFIG_UAV_MASTER_TICK_HZ)
#define TIMEBASE_SUPERFRAME_TICKS       CONFIG_UAV_SLOW_DIVIDER

/* Tick flags - để task biết domain nào được trigger */
#define TICK_FLAG_FAST                  (1 << 0)
#define TICK_FLAG_MEDIUM                (1 << 1)
#define TICK_FLAG_SLOW                  (1 << 2)
#define TICK_FLAG_ASYNC                 (1 << 3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @brief Domain index trong hệ thống multi-rate
 *
 * FAST:   8 kHz - IMU acquisition, gyro integration
 * MEDIUM: 1 kHz - EKF prediction, sensor preprocessing
 * SLOW:   100 Hz - Baro/Mag/GPS fusion, covariance update
 * ASYNC:  Event-driven - GPS parsing, telemetry, logging
 */
typedef enum {
    DOMAIN_FAST = 0,    /* 8 kHz */
    DOMAIN_MEDIUM = 1,  /* 1 kHz */
    DOMAIN_SLOW = 2,    /* 100 Hz */
    DOMAIN_ASYNC = 3,   /* Event-driven */
    DOMAIN_COUNT = 4
} timebase_domain_t;

/**
 * @brief Thống kê jitter của master tick
 */
typedef struct {
    uint32_t min_period_ns;     /* Period nhỏ nhất đo được */
    uint32_t max_period_ns;     /* Period lớn nhất đo được */
    uint64_t avg_period_ns;     /* Period trung bình (tích lũy) */
    uint32_t samples;           /* Số samples đã đo */
    uint32_t deadline_misses;   /* Số lần miss deadline */
} timebase_jitter_stats_t;

/**
 * @brief Trạng thái của timebase
 */
typedef struct {
    volatile uint64_t   tick_count;         /* Bộ đếm tick (monotonic) */
    volatile uint64_t   timestamp_us;       /* Timestamp tuyệt đối (µs) */
    volatile uint32_t   tick_phase;         /* Phase trong superframe (0-79) */
    volatile uint8_t    tick_flags;         /* Flags cho domains được trigger */

    sem_t               domain_sem[DOMAIN_COUNT];   /* Semaphores cho mỗi domain */

    bool                initialized;
    bool                running;

    /* Statistics */
    timebase_jitter_stats_t jitter;
} timebase_state_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Khởi tạo Master Timebase
 *
 * Setup hardware timer và bắt đầu tạo ticks.
 * PHẢI được gọi một lần duy nhất khi system boot.
 *
 * @return 0 nếu thành công, -errno nếu lỗi
 */
int timebase_init(void);

/**
 * @brief Dừng và cleanup timebase
 *
 * Dừng timer interrupt và giải phóng resources.
 * Gọi khi system shutdown.
 */
void timebase_deinit(void);

/**
 * @brief Kiểm tra timebase đã được khởi tạo chưa
 *
 * @return true nếu đã init, false nếu chưa
 */
bool timebase_is_initialized(void);

/**
 * @brief Lấy timestamp hiện tại (microseconds)
 *
 * Thread-safe. Có thể gọi từ bất kỳ context nào.
 *
 * @return Số microseconds kể từ khi timebase_init()
 */
uint64_t timebase_get_timestamp(void);

/**
 * @brief Lấy tick count hiện tại
 *
 * @return Số ticks kể từ khi timebase_init()
 */
uint64_t timebase_get_tick_count(void);

/**
 * @brief Lấy phase hiện tại trong superframe
 *
 * Phase chạy từ 0 đến (TIMEBASE_SUPERFRAME_TICKS - 1).
 *
 * @return Phase index (0-79 với cấu hình mặc định)
 */
uint32_t timebase_get_phase(void);

/**
 * @brief Đợi tick của một domain cụ thể
 *
 * Task sẽ block cho đến khi domain được trigger.
 * Đây là cách chính để synchronize task với master tick.
 *
 * @param domain Domain cần đợi
 * @return 0 nếu có tick, -EINTR nếu bị signal interrupt
 */
int timebase_wait_domain(timebase_domain_t domain);

/**
 * @brief Đợi tick với timeout
 *
 * @param domain Domain cần đợi
 * @param timeout_us Timeout tính bằng microseconds
 * @return 0 nếu có tick, -ETIMEDOUT nếu timeout, -EINTR nếu bị interrupt
 */
int timebase_wait_domain_timeout(timebase_domain_t domain, uint32_t timeout_us);

/**
 * @brief Signal async domain từ ISR hoặc external event
 *
 * Được sử dụng khi có async event (GPS data ready, DMA complete, etc.)
 * Thread-safe, có thể gọi từ ISR context.
 */
void timebase_signal_async(void);

/**
 * @brief Lấy thống kê jitter
 *
 * @param stats Pointer để nhận thống kê (output)
 */
void timebase_get_jitter_stats(timebase_jitter_stats_t *stats);

/**
 * @brief Reset thống kê jitter
 */
void timebase_reset_jitter_stats(void);

/**
 * @brief Lấy tần số thực tế của mỗi domain
 *
 * @param domain Domain cần query
 * @return Tần số tính bằng Hz
 */
uint32_t timebase_get_domain_hz(timebase_domain_t domain);

/**
 * @brief In trạng thái debug của timebase
 */
void timebase_print_status(void);

#ifdef __cplusplus
}
#endif

#endif /* __UAV_LIB_PLATFORM_TIMEBASE_H */
