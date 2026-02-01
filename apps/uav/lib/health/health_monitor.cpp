/****************************************************************************
 * apps/uav/lib/health/health_monitor.cpp
 *
 * HEALTH MONITOR - Implementation
 *
 ****************************************************************************/

#include "health_monitor.hpp"
#include <uav/lib/platform/hrt.h>

#include <stdio.h>
#include <string.h>
#include <syslog.h>

namespace uav {
namespace health {

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Global singleton instance */
static HealthMonitor g_health_monitor;

/****************************************************************************
 * Component names for debug output
 ****************************************************************************/

static const char* component_names[] = {
    "MASTER_TICK",
    "IMU_0",
    "IMU_1",
    "IMU_2",
    "IMU_3",
    "BARO",
    "MAG",
    "GPS",
    "EKF",
    "LOGGER"
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

HealthMonitor::HealthMonitor()
    : m_system_level(HealthLevel::NOMINAL)
    , m_init_time_us(0)
    , m_initialized(false)
{
    memset(m_components, 0, sizeof(m_components));
}

HealthMonitor::~HealthMonitor()
{
    deinit();
}

int HealthMonitor::init()
{
    if (m_initialized) {
        return 0;
    }

    /* Initialize all components */
    for (int i = 0; i < static_cast<int>(ComponentId::COMPONENT_COUNT); i++) {
        m_components[i].last_heartbeat_us = 0;
        m_components[i].missed_beats = 0;
        m_components[i].error_count = 0;
        m_components[i].total_beats = 0;
        m_components[i].functional = false;
        m_components[i].present = false;
        m_components[i].level = HealthLevel::NOMINAL;
    }

    /* Master tick is always present */
    m_components[static_cast<int>(ComponentId::MASTER_TICK)].present = true;

    m_init_time_us = get_time_us();
    m_system_level = HealthLevel::NOMINAL;
    m_initialized = true;

    syslog(LOG_INFO, "[health] Health monitor initialized\n");

    return 0;
}

void HealthMonitor::deinit()
{
    m_initialized = false;
}

void HealthMonitor::update()
{
    if (!m_initialized) {
        return;
    }

    uint64_t now = get_time_us();

    /* Check each component */
    for (int i = 0; i < static_cast<int>(ComponentId::COMPONENT_COUNT); i++) {
        ComponentHealth& comp = m_components[i];

        if (!comp.present) {
            continue;
        }

        /* Check if heartbeat expired */
        uint64_t elapsed_ms = (now - comp.last_heartbeat_us) / 1000;

        if (elapsed_ms > CONFIG_UAV_HEARTBEAT_TIMEOUT_MS) {
            comp.missed_beats++;

            /* Update level based on missed beats */
            update_component_level(static_cast<ComponentId>(i));

            /* Mark as non-functional if too many misses */
            if (comp.missed_beats >= CONFIG_UAV_MISSED_BEATS_FAIL) {
                if (comp.functional) {
                    syslog(LOG_WARNING, "[health] %s marked as non-functional\n",
                           component_names[i]);
                }
                comp.functional = false;
            }
        }
    }

    /* Recalculate system level */
    m_system_level = calculate_system_level();
}

void HealthMonitor::heartbeat(ComponentId id)
{
    if (!m_initialized || id >= ComponentId::COMPONENT_COUNT) {
        return;
    }

    int idx = static_cast<int>(id);
    ComponentHealth& comp = m_components[idx];

    comp.last_heartbeat_us = get_time_us();
    comp.missed_beats = 0;
    comp.total_beats++;
    comp.functional = true;
    comp.level = HealthLevel::NOMINAL;
}

void HealthMonitor::report_error(ComponentId id, int error_code)
{
    (void)error_code;

    if (!m_initialized || id >= ComponentId::COMPONENT_COUNT) {
        return;
    }

    int idx = static_cast<int>(id);
    m_components[idx].error_count++;

    syslog(LOG_WARNING, "[health] %s reported error (total: %lu)\n",
           component_names[idx], (unsigned long)m_components[idx].error_count);
}

void HealthMonitor::set_present(ComponentId id, bool present)
{
    if (!m_initialized || id >= ComponentId::COMPONENT_COUNT) {
        return;
    }

    int idx = static_cast<int>(id);
    m_components[idx].present = present;

    if (present) {
        m_components[idx].last_heartbeat_us = get_time_us();
        syslog(LOG_INFO, "[health] %s marked as present\n", component_names[idx]);
    }
}

const ComponentHealth& HealthMonitor::get_component_health(ComponentId id) const
{
    static ComponentHealth dummy = {};

    if (id >= ComponentId::COMPONENT_COUNT) {
        return dummy;
    }

    return m_components[static_cast<int>(id)];
}

SystemHealth HealthMonitor::get_system_health() const
{
    SystemHealth status = {};

    status.level = m_system_level;
    status.healthy_imus = get_healthy_imu_count();

    status.baro_ok = m_components[static_cast<int>(ComponentId::BARO)].functional;
    status.mag_ok = m_components[static_cast<int>(ComponentId::MAG)].functional;
    status.gps_ok = m_components[static_cast<int>(ComponentId::GPS)].functional;
    status.ekf_ok = m_components[static_cast<int>(ComponentId::EKF)].functional;

    status.uptime_ms = (uint32_t)((get_time_us() - m_init_time_us) / 1000);

    return status;
}

HealthLevel HealthMonitor::get_system_level() const
{
    return m_system_level;
}

bool HealthMonitor::should_failsafe() const
{
    /* Failsafe nếu:
     * - Không có IMU nào hoạt động
     * - Master tick chết
     * - EKF chết
     */

    if (get_healthy_imu_count() == 0) {
        return true;
    }

    if (!m_components[static_cast<int>(ComponentId::MASTER_TICK)].functional) {
        return true;
    }

    /* EKF không bắt buộc cho attitude-only mode */

    return m_system_level == HealthLevel::FAILSAFE;
}

uint8_t HealthMonitor::get_healthy_imu_count() const
{
    uint8_t count = 0;

    for (int i = static_cast<int>(ComponentId::IMU_0);
         i <= static_cast<int>(ComponentId::IMU_3); i++) {
        if (m_components[i].functional) {
            count++;
        }
    }

    return count;
}

uint8_t HealthMonitor::get_healthy_imu_mask() const
{
    uint8_t mask = 0;

    for (int i = static_cast<int>(ComponentId::IMU_0);
         i <= static_cast<int>(ComponentId::IMU_3); i++) {
        if (m_components[i].functional) {
            mask |= (1 << (i - static_cast<int>(ComponentId::IMU_0)));
        }
    }

    return mask;
}

void HealthMonitor::print_status() const
{
    printf("[health] System Status:\n");
    printf("  Level: ");

    switch (m_system_level) {
        case HealthLevel::NOMINAL:
            printf("NOMINAL\n");
            break;
        case HealthLevel::DEGRADED:
            printf("DEGRADED\n");
            break;
        case HealthLevel::CRITICAL:
            printf("CRITICAL\n");
            break;
        case HealthLevel::FAILSAFE:
            printf("FAILSAFE\n");
            break;
    }

    printf("  Healthy IMUs: %d/4\n", get_healthy_imu_count());

    printf("\n  Components:\n");

    for (int i = 0; i < static_cast<int>(ComponentId::COMPONENT_COUNT); i++) {
        const ComponentHealth& comp = m_components[i];

        if (!comp.present) {
            continue;
        }

        printf("    %-12s: %s (beats=%lu, miss=%lu, err=%lu)\n",
               component_names[i],
               comp.functional ? "OK" : "FAIL",
               (unsigned long)comp.total_beats,
               (unsigned long)comp.missed_beats,
               (unsigned long)comp.error_count);
    }
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

HealthLevel HealthMonitor::calculate_system_level() const
{
    /* Master tick chết = FAILSAFE */
    if (!m_components[static_cast<int>(ComponentId::MASTER_TICK)].functional) {
        return HealthLevel::FAILSAFE;
    }

    uint8_t healthy_imus = get_healthy_imu_count();

    /* Không có IMU = FAILSAFE */
    if (healthy_imus == 0) {
        return HealthLevel::FAILSAFE;
    }

    /* Chỉ còn 1 IMU = CRITICAL */
    if (healthy_imus == 1) {
        return HealthLevel::CRITICAL;
    }

    /* 2 IMU = DEGRADED */
    if (healthy_imus == 2) {
        return HealthLevel::DEGRADED;
    }

    /* 3 IMU = DEGRADED nhẹ */
    if (healthy_imus == 3) {
        /* Có thể còn NOMINAL nếu các sensor khác OK */
        bool sensors_ok =
            m_components[static_cast<int>(ComponentId::BARO)].functional &&
            m_components[static_cast<int>(ComponentId::MAG)].functional;

        return sensors_ok ? HealthLevel::NOMINAL : HealthLevel::DEGRADED;
    }

    /* 4 IMU hoạt động */
    return HealthLevel::NOMINAL;
}

bool HealthMonitor::is_component_expired(ComponentId id) const
{
    if (id >= ComponentId::COMPONENT_COUNT) {
        return true;
    }

    const ComponentHealth& comp = m_components[static_cast<int>(id)];

    if (!comp.present) {
        return true;
    }

    uint64_t elapsed_ms = (get_time_us() - comp.last_heartbeat_us) / 1000;
    return elapsed_ms > CONFIG_UAV_HEARTBEAT_TIMEOUT_MS;
}

void HealthMonitor::update_component_level(ComponentId id)
{
    if (id >= ComponentId::COMPONENT_COUNT) {
        return;
    }

    ComponentHealth& comp = m_components[static_cast<int>(id)];

    if (comp.missed_beats >= CONFIG_UAV_MISSED_BEATS_FAIL) {
        comp.level = HealthLevel::FAILSAFE;
    } else if (comp.missed_beats >= CONFIG_UAV_MISSED_BEATS_WARN) {
        comp.level = HealthLevel::DEGRADED;
    } else {
        comp.level = HealthLevel::NOMINAL;
    }
}

uint64_t HealthMonitor::get_time_us() const
{
    return hrt_absolute_time();
}

/****************************************************************************
 * Global Instance Access
 ****************************************************************************/

HealthMonitor& get_health_monitor()
{
    return g_health_monitor;
}

} /* namespace health */
} /* namespace uav */
