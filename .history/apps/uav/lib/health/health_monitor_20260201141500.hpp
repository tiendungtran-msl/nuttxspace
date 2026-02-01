/****************************************************************************
 * apps/uav/lib/health/health_monitor.hpp
 *
 * HEALTH MONITOR - Giám sát sức khỏe hệ thống
 *
 * MỤC ĐÍCH:
 * - Theo dõi tình trạng của tất cả components
 * - Phát hiện lỗi và degradation
 * - Graceful degradation khi có component fail
 * - Cung cấp thông tin cho flight controller
 *
 * THIẾT KẾ:
 * - Mỗi component được track qua ComponentHealth struct
 * - Heartbeat mechanism để detect stall/hang
 * - Multi-level health status (NOMINAL → DEGRADED → CRITICAL → FAILSAFE)
 * - Independent watchdog để detect master tick failure
 *
 * SỬ DỤNG:
 *   HealthMonitor monitor;
 *   monitor.init();
 *
 *   // Components gọi heartbeat định kỳ
 *   monitor.heartbeat(COMPONENT_IMU_0);
 *
 *   // Main loop check system health
 *   if (monitor.get_system_level() == HealthLevel::CRITICAL) {
 *       enter_failsafe();
 *   }
 *
 ****************************************************************************/

#ifndef __UAV_LIB_HEALTH_HEALTH_MONITOR_HPP
#define __UAV_LIB_HEALTH_HEALTH_MONITOR_HPP

#include <stdint.h>
#include <stdbool.h>

namespace uav {
namespace health {

/****************************************************************************
 * Configuration
 ****************************************************************************/

#ifndef CONFIG_UAV_HEALTH_CHECK_INTERVAL_MS
#define CONFIG_UAV_HEALTH_CHECK_INTERVAL_MS     100     /* 10 Hz */
#endif

#ifndef CONFIG_UAV_HEARTBEAT_TIMEOUT_MS
#define CONFIG_UAV_HEARTBEAT_TIMEOUT_MS         50      /* Miss threshold */
#endif

#ifndef CONFIG_UAV_MISSED_BEATS_WARN
#define CONFIG_UAV_MISSED_BEATS_WARN            3
#endif

#ifndef CONFIG_UAV_MISSED_BEATS_FAIL
#define CONFIG_UAV_MISSED_BEATS_FAIL            10
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @brief Mức độ sức khỏe hệ thống
 */
enum class HealthLevel : uint8_t {
    NOMINAL,        /* Tất cả hoạt động bình thường */
    DEGRADED,       /* Một số redundancy bị mất nhưng vẫn bay được */
    CRITICAL,       /* Tối thiểu chức năng - nên hạ cánh */
    FAILSAFE        /* Chế độ an toàn - chỉ giữ thăng bằng */
};

/**
 * @brief ID của các components được monitor
 */
enum class ComponentId : uint8_t {
    MASTER_TICK = 0,
    IMU_0,
    IMU_1,
    IMU_2,
    IMU_3,
    BARO,
    MAG,
    GPS,
    EKF,
    LOGGER,
    COMPONENT_COUNT
};

/**
 * @brief Trạng thái của một component
 */
struct ComponentHealth {
    uint64_t    last_heartbeat_us;  /* Timestamp của heartbeat gần nhất */
    uint32_t    missed_beats;       /* Số heartbeat bị miss liên tiếp */
    uint32_t    error_count;        /* Tổng số lỗi */
    uint32_t    total_beats;        /* Tổng số heartbeats */
    bool        functional;         /* Component có hoạt động không */
    bool        present;            /* Component có tồn tại không */
    HealthLevel level;              /* Mức độ sức khỏe */
};

/**
 * @brief Trạng thái tổng thể của hệ thống
 */
struct SystemHealth {
    HealthLevel     level;              /* Mức độ sức khỏe tổng thể */
    uint8_t         healthy_imus;       /* Số IMU đang hoạt động */
    bool            baro_ok;
    bool            mag_ok;
    bool            gps_ok;
    bool            ekf_ok;
    uint32_t        uptime_ms;          /* Thời gian hoạt động */
};

/****************************************************************************
 * HealthMonitor Class
 ****************************************************************************/

class HealthMonitor {
public:
    HealthMonitor();
    ~HealthMonitor();

    /* Delete copy/move */
    HealthMonitor(const HealthMonitor&) = delete;
    HealthMonitor& operator=(const HealthMonitor&) = delete;

    /**
     * @brief Khởi tạo health monitor
     * @return 0 nếu thành công
     */
    int init();

    /**
     * @brief Deinit và cleanup
     */
    void deinit();

    /**
     * @brief Update - gọi định kỳ từ health check task
     *
     * Kiểm tra heartbeats, update health levels.
     */
    void update();

    /**
     * @brief Component gửi heartbeat
     *
     * @param id Component ID
     */
    void heartbeat(ComponentId id);

    /**
     * @brief Component báo lỗi
     *
     * @param id Component ID
     * @param error_code Error code (optional)
     */
    void report_error(ComponentId id, int error_code = 0);

    /**
     * @brief Đánh dấu component là present/absent
     *
     * @param id Component ID
     * @param present true nếu component tồn tại
     */
    void set_present(ComponentId id, bool present);

    /**
     * @brief Lấy health của một component
     *
     * @param id Component ID
     * @return ComponentHealth struct
     */
    const ComponentHealth& get_component_health(ComponentId id) const;

    /**
     * @brief Lấy tổng thể system health
     *
     * @return SystemHealth struct
     */
    SystemHealth get_system_health() const;

    /**
     * @brief Lấy health level của system
     * @return HealthLevel
     */
    HealthLevel get_system_level() const;

    /**
     * @brief Kiểm tra có nên enter failsafe không
     * @return true nếu nên failsafe
     */
    bool should_failsafe() const;

    /**
     * @brief Lấy số IMU đang hoạt động
     * @return Số IMU functional
     */
    uint8_t get_healthy_imu_count() const;

    /**
     * @brief Lấy bitmask của IMU hoạt động
     * @return Bitmask (bit 0 = IMU0, bit 1 = IMU1, etc.)
     */
    uint8_t get_healthy_imu_mask() const;

    /**
     * @brief In status debug
     */
    void print_status() const;

private:
    /* Tính toán system health level từ component states */
    HealthLevel calculate_system_level() const;

    /* Check một component có expired không */
    bool is_component_expired(ComponentId id) const;

    /* Update level của một component */
    void update_component_level(ComponentId id);

    /* Get current time helper */
    uint64_t get_time_us() const;

    /* Component health array */
    ComponentHealth m_components[static_cast<int>(ComponentId::COMPONENT_COUNT)];

    /* Cached system level */
    HealthLevel m_system_level;

    /* Init timestamp */
    uint64_t m_init_time_us;

    /* Initialized flag */
    bool m_initialized;
};

/****************************************************************************
 * Global instance (singleton pattern)
 ****************************************************************************/

/**
 * @brief Lấy global HealthMonitor instance
 * @return Reference đến singleton
 */
HealthMonitor& get_health_monitor();

} /* namespace health */
} /* namespace uav */

#endif /* __UAV_LIB_HEALTH_HEALTH_MONITOR_HPP */
