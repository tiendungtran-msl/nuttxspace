# lib/health - Health Monitor

## 📋 Tổng Quan

Module `health` cung cấp **hệ thống giám sát sức khỏe** cho toàn bộ UAV:
- Theo dõi tình trạng tất cả components
- Phát hiện lỗi qua heartbeat mechanism
- Graceful degradation khi có component fail
- Cung cấp thông tin cho flight controller quyết định failsafe

---

## 📁 Files

```
health/
├── Makefile
├── health_monitor.hpp    # API và định nghĩa
└── health_monitor.cpp    # Implementation
```

---

## 🎯 Thiết Kế

### Kiến Trúc

```
┌─────────────────────────────────────────────────────────────────────┐
│                        HealthMonitor                                 │
│                                                                      │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │                    ComponentHealth Array                        │ │
│  │ ┌──────────┬──────────┬──────────┬──────────┬─────────────────┐ │ │
│  │ │ IMU_0    │ IMU_1    │ IMU_2    │ IMU_3    │ BARO, MAG, GPS..│ │ │
│  │ └──────────┴──────────┴──────────┴──────────┴─────────────────┘ │ │
│  └────────────────────────────────────────────────────────────────┘ │
│                                                                      │
│  ┌───────────────────┐  ┌───────────────────┐                       │
│  │ heartbeat()       │  │ check_timeouts()  │                       │
│  │ - Update timestamp│  │ - Detect stalls   │                       │
│  │ - Reset counters  │  │ - Update status   │                       │
│  └───────────────────┘  └───────────────────┘                       │
│                                                                      │
│  ┌───────────────────────────────────────────────────────────────┐  │
│  │                    System Health Level                         │  │
│  │        NOMINAL → DEGRADED → CRITICAL → FAILSAFE               │  │
│  └───────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

### Heartbeat Mechanism

```
Component gọi heartbeat()     HealthMonitor
        │                           │
        │   heartbeat(IMU_0)        │
        │──────────────────────────▶│
        │                           │ ─┐ Update timestamp
        │                           │  │ Reset missed_beats
        │                           │ ─┘
        │                           │
        │         (50ms timeout)    │
        │                           │
        │                           │ ─┐ check_timeouts()
        │                           │  │ Detect missed beats
        │                           │ ─┘
        │                           │
        ▼                           ▼

Nếu component không gọi heartbeat trong 50ms:
  - missed_beats++
  - Sau 3 misses: DEGRADED warning
  - Sau 10 misses: Component marked FAILED
```

---

## 📊 Health Levels

| Level | Điều Kiện | Hành Động |
|-------|-----------|-----------|
| **NOMINAL** | Tất cả components OK | Bay bình thường |
| **DEGRADED** | ≥1 redundant component fail | Cảnh báo, tiếp tục bay |
| **CRITICAL** | Critical component fail | Yêu cầu hạ cánh |
| **FAILSAFE** | Multiple critical fails | Chỉ giữ thăng bằng |

### Component Criticality

```cpp
// Critical components - ảnh hưởng trực tiếp đến an toàn
MASTER_TICK   // System heartbeat
IMU_0/1/2/3   // Ít nhất 1 IMU phải hoạt động
EKF           // State estimation

// Important components - giảm chức năng nếu fail
BARO          // Altitude hold degraded
MAG           // Heading degraded
GPS           // Position hold không khả dụng

// Non-critical
LOGGER        // Không ghi log nhưng vẫn bay được
```

---

## 🔧 API

### Initialization

```cpp
#include <uav/lib/health/health_monitor.hpp>

using namespace uav::health;

HealthMonitor monitor;

// Khởi tạo
int ret = monitor.init();
if (ret != 0) {
    // Handle error
}
```

### Component Heartbeat

```cpp
// Gọi định kỳ từ mỗi component
void sensors_main() {
    while (!should_exit) {
        // Đọc IMU
        read_imu(0, &data);
        
        // Báo heartbeat cho IMU_0
        monitor.heartbeat(ComponentId::IMU_0);
        
        usleep(1000);
    }
}
```

### Report Error

```cpp
// Báo lỗi khi detect vấn đề
if (spi_read_failed) {
    monitor.report_error(ComponentId::IMU_1);
}
```

### Query Status

```cpp
// Lấy system health level
HealthLevel level = monitor.get_system_level();

switch (level) {
    case HealthLevel::NOMINAL:
        // OK
        break;
    case HealthLevel::DEGRADED:
        // Cảnh báo pilot
        break;
    case HealthLevel::CRITICAL:
        // Yêu cầu hạ cánh
        break;
    case HealthLevel::FAILSAFE:
        // Emergency landing
        break;
}

// Lấy component status
ComponentHealth status = monitor.get_component_health(ComponentId::IMU_0);
if (!status.functional) {
    // IMU_0 bị fail
}

// Check specific component
bool imu_ok = monitor.is_functional(ComponentId::IMU_0);
```

### Get Statistics

```cpp
// Số components functional
uint8_t num_ok = monitor.get_functional_count();

// Healthy IMU mask
uint8_t imu_mask = monitor.get_healthy_imu_mask();
// 0b1111 = all 4 OK
// 0b1011 = IMU_2 failed
// 0b0001 = only IMU_0 OK
```

---

## 📐 Configuration

Trong `Kconfig` hoặc `defconfig`:

```
# Check interval (10 Hz default)
CONFIG_UAV_HEALTH_CHECK_INTERVAL_MS=100

# Heartbeat timeout
CONFIG_UAV_HEARTBEAT_TIMEOUT_MS=50

# Số misses trước khi cảnh báo
CONFIG_UAV_MISSED_BEATS_WARN=3

# Số misses trước khi fail
CONFIG_UAV_MISSED_BEATS_FAIL=10
```

---

## 🔄 State Machine

```
               ┌───────────────────────────────────────┐
               │                                       │
               ▼                                       │
         ┌──────────┐    any fail    ┌──────────────┐  │
─────────▶│ NOMINAL  │──────────────▶│  DEGRADED    │──┘
         └──────────┘                └──────────────┘
               │                           │
               │                           │
               │ critical fail             │ critical fail
               ▼                           ▼
         ┌──────────────────────────────────────────┐
         │              CRITICAL                     │
         └──────────────────────────────────────────┘
                           │
                           │ multiple critical fail
                           ▼
         ┌──────────────────────────────────────────┐
         │              FAILSAFE                     │
         └──────────────────────────────────────────┘
```

### Recovery

```cpp
// Component có thể recover nếu:
// 1. Heartbeat được resume
// 2. Liên tiếp N beats OK
// 3. Không còn lỗi

void component_recovery() {
    if (status.missed_beats == 0 &&
        consecutive_ok >= RECOVERY_THRESHOLD) {
        status.functional = true;
        // Recalculate system level
    }
}
```

---

## 📊 Data Structures

### ComponentHealth

```cpp
struct ComponentHealth {
    uint64_t last_heartbeat_us;   // Timestamp heartbeat cuối
    uint32_t missed_beats;        // Số beats miss liên tiếp
    uint32_t error_count;         // Tổng số lỗi
    uint32_t total_beats;         // Tổng số heartbeats
    bool     functional;          // Component hoạt động?
};
```

### ComponentId Enum

```cpp
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
    COMPONENT_COUNT   // = 10
};
```

---

## 💡 Best Practices

### 1. Heartbeat Frequency

```cpp
// Component nên gọi heartbeat ít nhất 2× so với timeout
// Timeout = 50ms → heartbeat mỗi 20-25ms
void fast_component() {
    while (running) {
        do_work();
        monitor.heartbeat(MY_ID);
        usleep(20000);  // 50 Hz
    }
}
```

### 2. Error Reporting

```cpp
// Phân biệt transient vs persistent errors
if (crc_error) {
    // Transient - chỉ report, không panic
    monitor.report_error(IMU_0);
    retry_count++;
    if (retry_count > MAX_RETRIES) {
        // Persistent - disable component
        monitor.mark_failed(IMU_0);
    }
}
```

### 3. Graceful Degradation

```cpp
// Sensors app: adapt to available IMUs
uint8_t mask = monitor.get_healthy_imu_mask();
int healthy = __builtin_popcount(mask);

if (healthy >= 3) {
    fusion.set_mode(FusionMode::VOTING);
} else if (healthy >= 2) {
    fusion.set_mode(FusionMode::WEIGHTED);
} else if (healthy >= 1) {
    fusion.set_mode(FusionMode::PRIMARY);
} else {
    // No IMU - emergency!
    trigger_failsafe();
}
```

---

## 🔗 Liên Kết

- **Usage:** `/apps/uav/sensors_app/`, `/apps/uav/state_app/`
- **State Machine:** `/apps/uav/state_app/README.md`
- **Architecture:** `/apps/uav/ARCHITECTURE.md`
