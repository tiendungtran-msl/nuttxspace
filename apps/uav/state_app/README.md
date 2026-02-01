# State Application - Quản lý trạng thái UAV

## Tổng quan

State App chịu trách nhiệm quản lý trạng thái hoạt động của UAV, bao gồm:
- Arming/Disarming
- Flight mode selection
- Failsafe detection và handling
- State machine tổng thể

## Vị trí trong Pipeline

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│ sensors_app │────►│estimator_app│────►│  state_app  │
│   (250 Hz)  │     │  (poll-based)│    │   (50 Hz)   │
└─────────────┘     └─────────────┘     └─────────────┘
       │                   │                   │
       │                   │                   ▼
       │                   │            ┌─────────────┐
       │                   └───────────►│ controller  │
       │                                │  (future)   │
       │                                └─────────────┘
       │                                       ▲
       └───────────────────────────────────────┘
                    sensor_combined
```

## Chức năng chính

### 1. State Machine

```
                  ┌─────────────────────────────────────┐
                  │                                     │
                  ▼                                     │
    ┌─────────────────────┐                            │
    │     UNINITALIZED    │                            │
    │   (chờ sensors ok)  │                            │
    └──────────┬──────────┘                            │
               │ sensors_ok                            │
               ▼                                       │
    ┌─────────────────────┐                            │
    │      STANDBY        │◄───────────────────────────┤
    │  (sẵn sàng arm)     │                            │
    └──────────┬──────────┘                            │
               │ arm_cmd && checks_pass                │
               ▼                                       │
    ┌─────────────────────┐    disarm                 │
    │       ARMED         │────────────────────────────┘
    │  (motors enabled)   │
    └──────────┬──────────┘
               │ takeoff_cmd
               ▼
    ┌─────────────────────┐
    │     IN_FLIGHT       │
    │  (đang bay)         │
    └──────────┬──────────┘
               │ land_complete
               ▼
    ┌─────────────────────┐
    │      LANDED         │
    │  (auto disarm)      │
    └─────────────────────┘
```

### 2. Preflight Checks

Trước khi arm, kiểm tra:
- [ ] IMU calibrated và valid
- [ ] Mag calibrated (nếu dùng)
- [ ] GPS có fix (nếu cần)
- [ ] Battery đủ
- [ ] RC link (nếu có)
- [ ] Estimator converged

### 3. Failsafe Handling

| Failsafe        | Action                    |
|-----------------|---------------------------|
| IMU timeout     | Emergency disarm          |
| Estimator fail  | Switch to attitude mode   |
| GPS lost        | Hold position (nếu có)    |
| RC lost         | Auto land                 |
| Low battery     | Auto RTL                  |

### 4. Flight Modes

```
   ┌─────────────────────────────────────────────────────────┐
   │                    MANUAL MODES                         │
   ├─────────────────────────────────────────────────────────┤
   │ ACRO      - Direct rate control                         │
   │ MANUAL    - Direct attitude control                     │
   │ STABILIZE - Attitude hold, manual throttle              │
   │ ALTITUDE  - Attitude + altitude hold                    │
   ├─────────────────────────────────────────────────────────┤
   │                   ASSISTED MODES                        │
   ├─────────────────────────────────────────────────────────┤
   │ POSHOLD   - Position + altitude hold                    │
   │ LOITER    - Circle around point                         │
   │ RTL       - Return to launch                            │
   ├─────────────────────────────────────────────────────────┤
   │                      AUTO MODES                         │
   ├─────────────────────────────────────────────────────────┤
   │ AUTO      - Follow mission waypoints                    │
   │ TAKEOFF   - Automatic takeoff                           │
   │ LAND      - Automatic landing                           │
   └─────────────────────────────────────────────────────────┘
```

## Thiết kế

### Rate-based Loop

Khác với estimator_app (event-driven), state_app chạy theo rate cố định:

```cpp
while (!g_should_exit) {
    uint64_t start = hrt_absolute_time();

    // Update state machine
    update_state_machine();

    // Check failsafes
    check_failsafes();

    // Publish state
    publish_vehicle_state();

    // Sleep to maintain 50 Hz
    uint64_t elapsed = hrt_absolute_time() - start;
    if (elapsed < LOOP_PERIOD_US) {
        usleep(LOOP_PERIOD_US - elapsed);
    }
}
```

### Publisher/Subscriber

**Subscribe:**
- `vehicle_attitude` - từ estimator
- `vehicle_local_position` - từ estimator
- `sensor_imu` - để check timeout
- `vehicle_command` (future) - từ RC/GCS

**Publish:**
- `vehicle_state` - trạng thái hiện tại cho controller
- `vehicle_status` (future) - detailed status for GCS

## Cấu trúc file

```
state_app/
├── README.md           # File này
├── state_main.cpp      # Entry point, main loop
├── state_machine.hpp   # State machine definition
├── preflight_check.hpp # Pre-arm checks
├── failsafe.hpp        # Failsafe logic
├── Makefile
└── Kconfig
```

## Timing Requirements

| Metric           | Target      | Max         |
|------------------|-------------|-------------|
| Loop rate        | 50 Hz       | -           |
| Loop jitter      | < 1 ms      | 2 ms        |
| State transition | < 20 ms     | 50 ms       |
| Failsafe trigger | < 100 ms    | 200 ms      |

## Usage

```bash
# Khởi động
nsh> state start

# Xem trạng thái
nsh> state status

# Arm (khi có đủ điều kiện)
nsh> state arm

# Disarm
nsh> state disarm

# Dừng app
nsh> state stop
```

## Safety Notes

⚠️ **CRITICAL**: State app là safety-critical!

1. **Watchdog**: Tự động disarm nếu không nhận được IMU data
2. **Arming checks**: Không bỏ qua pre-arm checks
3. **Failsafe priority**: Luôn ưu tiên safety over mission
4. **Logging**: Log tất cả state transitions
