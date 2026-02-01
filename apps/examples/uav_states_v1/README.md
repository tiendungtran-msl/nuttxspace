# UAV Attitude Estimation - Modular Pub/Sub Architecture

## Overview

This application implements a UAV attitude estimation system using a modular
pub/sub architecture inspired by PX4's uORB. The design allows for easy
expansion with additional sensors (magnetometer, barometer, GPS) and processing
modules (EKF2, logger, controller).

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Main Application                                │
│                    (Coordinator / Dispatcher)                           │
└─────────────────────────────────────────────────────────────────────────┘
         │                       │                         │
         ▼                       ▼                         ▼
   ┌──────────┐          ┌───────────────┐         ┌─────────────┐
   │ Sensors  │ ──────►  │  Estimator    │ ──────► │  Display    │
   │ Module   │ sensor_  │  Module       │  att    │  (syslog)   │
   │ (Thread) │ imu      │  (called)     │         │             │
   └──────────┘          └───────────────┘         └─────────────┘
        │                       │
        │                       │
   ┌────▼───────────────────────▼────┐
   │         uORB Topics             │
   │  sensor_imu[4]  vehicle_att[4]  │
   └─────────────────────────────────┘
```

## Directory Structure

```
uav_states_v1/
├── main.cpp                    # Main coordinator (entry point)
├── uorb/                       # Pub/Sub message system
│   ├── uorb.hpp               # Topic & Subscription templates
│   └── topics.hpp             # Message definitions (sensor_imu_s, etc.)
├── modules/                    # Independent processing modules
│   ├── sensors/               # Sensor data acquisition
│   │   ├── imu_module.hpp     # IMU manager interface
│   │   └── imu_module.cpp     # IMU thread (PRODUCER)
│   └── estimator/             # Attitude estimation
│       ├── attitude_module.hpp
│       └── attitude_module.cpp # Estimator (CONSUMER → PRODUCER)
├── drivers/                    # Hardware drivers
│   └── imu/icm42688p/         # ICM-42688-P SPI driver
├── lib/                        # Shared libraries
│   ├── attitude_estimator/    # Quaternion complementary filter
│   ├── drivers/spi/           # SPI abstraction layer
│   └── utils/                 # Debug, critical sections
├── calibration/               # Sensor calibration
└── platforms/                 # Platform abstraction
    ├── boards/spi_config.h    # Board-specific SPI config
    └── nuttx/hrt/             # High-resolution timer
```

## Data Flow (Pub/Sub)

1. **ImuModule** (separate thread @ 100Hz):
   - Reads ICM-42688-P sensors via SPI
   - Publishes `sensor_imu_s` to uORB topics (one per sensor)

2. **AttitudeModule** (main thread):
   - Subscribes to `sensor_imu_s` topics
   - Runs quaternion complementary filter
   - Publishes `vehicle_attitude_s` topics

3. **Main** (coordinator):
   - Calls `AttitudeModule.update()` each loop
   - Reads attitude and displays via syslog (non-blocking)

## Key Design Decisions

### Why Pub/Sub?
- **Decoupling**: Sensor I/O isolated from processing
- **Modularity**: Easy to add new modules without touching existing code
- **Timing**: Each module can run at its own rate
- **Debugging**: Topics can be logged/inspected independently

### Why syslog for Display?
- Non-blocking (asynchronous buffering)
- printf blocks on serial output (~10ms per call)
- Critical for maintaining 100Hz loop timing

### Why Single Sensor Thread?
- All SPI transactions in one thread → no bus contention
- Simpler than per-sensor threads with locking
- Sequential reads are fast enough at 100Hz

## Future Expansion

### Adding Magnetometer (BMM150)
1. Create `modules/sensors/mag_module.hpp`
2. Add `sensor_mag_s` to `uorb/topics.hpp`
3. Subscribe in `AttitudeModule` or create new `HeadingModule`

### Adding Barometer (MS5611)
1. Create `modules/sensors/baro_module.hpp`
2. Add `sensor_baro_s` to `uorb/topics.hpp`
3. Use for altitude estimation in EKF2

### Adding EKF2
1. Create `modules/ekf2/ekf2_module.hpp`
2. Subscribe to: `sensor_imu_s`, `sensor_mag_s`, `sensor_baro_s`, `sensor_gps_s`
3. Publish: `vehicle_local_position_s`, `vehicle_attitude_s`

### Adding SD Card Logger
1. Create `modules/logger/logger_module.hpp`
2. Subscribe to all relevant topics
3. Write to SD card at configurable rate

## Building

```bash
cd nuttx
make -j8
```

## Running

```
nsh> uav_attitude
```

Press Ctrl+C to stop.

## Message Types

| Topic | Publisher | Subscriber | Rate |
|-------|-----------|------------|------|
| `sensor_imu_s` | ImuModule | AttitudeModule | 100Hz |
| `vehicle_attitude_s` | AttitudeModule | Main (display) | 100Hz |
| `estimator_status_s` | AttitudeModule | Logger (future) | 1Hz |

## Configuration

Edit constants in respective module headers:
- `modules/sensors/imu_module.hpp`: `SENSOR_RATE_HZ`, `CALIBRATION_TIME_MS`
- `main.cpp`: `MAIN_LOOP_RATE_HZ`, `DISPLAY_RATE_HZ`
- `uorb/uorb.hpp`: Topic ring buffer depth
