/****************************************************************************
 * apps/uav/uorb/topics/system_status.hpp
 *
 * SYSTEM STATUS - Trạng thái tổng thể của hệ thống
 *
 * MỤC ĐÍCH:
 * - Báo cáo health level của hệ thống
 * - Thông tin uptime, CPU load, memory
 * - Dùng cho monitoring và telemetry
 *
 ****************************************************************************/

#ifndef __UAV_UORB_TOPICS_SYSTEM_STATUS_HPP
#define __UAV_UORB_TOPICS_SYSTEM_STATUS_HPP

#include <uav/uorb/orb_defines.hpp>
#include <stdint.h>

/**
 * @brief Mức độ sức khỏe hệ thống
 */
enum class SystemHealthLevel : uint8_t {
    NOMINAL = 0,        /**< Tất cả hoạt động bình thường */
    DEGRADED = 1,       /**< Một số redundancy bị mất */
    CRITICAL = 2,       /**< Chức năng tối thiểu */
    FAILSAFE = 3        /**< Chế độ an toàn */
};

/**
 * @brief Trạng thái hệ thống
 */
struct system_status_s {
    uint64_t timestamp_us;          /**< Timestamp */

    /* Health */
    uint8_t health_level;           /**< SystemHealthLevel enum */
    uint8_t healthy_imus;           /**< Số IMU đang hoạt động */
    bool baro_ok;
    bool mag_ok;
    bool gps_ok;
    bool ekf_ok;
    bool timebase_ok;

    /* Timing */
    uint32_t uptime_ms;             /**< Thời gian hoạt động */
    uint32_t master_tick_count;     /**< Số master ticks */
    uint32_t deadline_misses;       /**< Số deadline miss */

    /* Timing jitter (nanoseconds) */
    uint32_t jitter_min_ns;
    uint32_t jitter_max_ns;
    uint32_t jitter_avg_ns;

    /* Performance */
    float cpu_load_percent;         /**< CPU load (0-100) */
    uint32_t free_heap_bytes;       /**< Free heap memory */
    uint32_t stack_usage_bytes;     /**< Stack usage */

    /* Sensor rates (actual measured) */
    float imu_rate_hz;
    float baro_rate_hz;
    float mag_rate_hz;
    float gps_rate_hz;
};

ORB_DECLARE(system_status);

#endif /* __UAV_UORB_TOPICS_SYSTEM_STATUS_HPP */
