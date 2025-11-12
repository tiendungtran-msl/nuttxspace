# IMU System Application

Complete 9-DOF IMU system with sensor fusion for NuttX RTOS on STM32H743.

## Features

- **Multi-sensor Support**: 4x ICM42688P IMU + 1x BMM150 Magnetometer
- **Real-time Sensor Fusion**: Madgwick AHRS algorithm for orientation estimation
- **High-rate Data Acquisition**: 1 kHz sensor reading, 100 Hz fusion update
- **LED Status Indication**: Visual feedback for system state
- **Calibration Support**: Gyroscope and magnetometer calibration
- **Thread-safe Design**: POSIX threads with mutex protection
- **Lock-free Queues**: Efficient inter-task communication

## Architecture

```
┌─────────────────┐
│  Main Thread    │
│  (CLI & Status) │
└────────┬────────┘
         │
    ┌────┴────────────────┬──────────────┐
    │                     │              │
┌───▼────────┐   ┌───────▼──────┐  ┌───▼──────┐
│ Sensor     │   │ Fusion       │  │ LED      │
│ Task       │──▶│ Task         │  │ Task     │
│ (1000 Hz)  │   │ (100 Hz)     │  │ (10 Hz)  │
└────┬───────┘   └──────┬───────┘  └──────────┘
     │                  │
     │   Sensor Queue   │   Fusion Queue
     └─────────────────▶│
                        │
                   ┌────▼────┐
                   │Madgwick │
                   │ Filter  │
                   └─────────┘
```

## Directory Structure

```
apps/examples/imu_system/
├── imu_system_main.c          # Main entry point
├── Kconfig                     # Configuration options
├── Makefile                    # Build system
├── drivers/
│   ├── spi/                   # SPI bus manager
│   ├── icm42688p/             # ICM42688P driver (4 instances)
│   ├── bmm150/                # BMM150 magnetometer driver
│   └── led/                   # LED status driver
├── algorithms/
│   ├── sensor_fusion.c        # Madgwick AHRS filter
│   └── orientation.c          # Heading calculation
├── tasks/
│   ├── sensor_task.c          # Sensor reading task
│   ├── fusion_task.c          # Sensor fusion task
│   └── led_task.c             # LED status task
└── utils/
    ├── config.h               # System configuration
    └── data_queue.c           # Lock-free circular buffer
```

## Hardware Configuration

### SPI1 Pin Assignment

| Device | CS Pin | Description |
|--------|--------|-------------|
| ICM0   | PA4    | ICM42688P #0 |
| ICM1   | PD11   | ICM42688P #1 |
| ICM2   | PD12   | ICM42688P #2 |
| ICM3   | PD13   | ICM42688P #3 |
| BMM150 | PD14   | Magnetometer |

### SPI Configuration

- **Bus**: SPI1
- **Mode**: MODE3 (CPOL=1, CPHA=1)
- **Frequency**: 10 MHz
- **Bit Order**: MSB First

## Building

1. Configure NuttX for weact-stm32h743 board:
```bash
cd nuttx
./tools/configure.sh weact-stm32h743:nsh
```

2. Enable the IMU system application:
```bash
make menuconfig
# Navigate to: Application Configuration -> Examples -> IMU System
# Enable "IMU System with 4x ICM42688P + BMM150"
```

3. Build:
```bash
make
```

## Usage

### Basic Operation

```bash
nsh> imu_system
```

### Calibration Mode

```bash
nsh> imu_system -c
# Keep sensors stationary during gyro calibration
# Rotate sensors in all directions during mag calibration
```

### Debug Mode

```bash
nsh> imu_system -d
# Shows quaternions and queue status
```

### Custom Sample Rate

```bash
nsh> imu_system -r 500
# Sets sensor rate to 500 Hz (default is 1000 Hz)
```

## Output

The application prints orientation data every second:

```
IMU System Starting...
Configuration:
  Sensors: 4 x ICM42688P + 1 x BMM150
  Sensor Rate: 1000 Hz
  Fusion Rate: 100 Hz
IMU System running. Press Ctrl+C to stop.

Orientation - Roll:   2.34° Pitch:  -1.23° Yaw:  45.67° Heading:  45.23°
Orientation - Roll:   2.35° Pitch:  -1.24° Yaw:  45.68° Heading:  45.24°
...
```

## Configuration Options

See `utils/config.h` for system parameters:

- `IMU_NUM_ICM42688P`: Number of ICM sensors (4)
- `IMU_SENSOR_RATE_HZ`: Sensor reading rate (1000 Hz)
- `IMU_FUSION_RATE_HZ`: Fusion update rate (100 Hz)
- `IMU_MADGWICK_BETA`: Filter gain (0.1)
- Task priorities and stack sizes
- Queue depths

## LED Status Patterns

| Pattern      | Meaning |
|--------------|---------|
| Solid        | Normal operation |
| Slow Blink   | Calibration in progress |
| Fast Blink   | Error condition (queue overflow) |
| Off          | No data / System stopped |

## Performance

- **Memory Usage**: ~96 KB total
  - Sensor queue: 64 samples × 1.5 KB = 96 KB
  - Fusion queue: 32 samples × 64 B = 2 KB
  - Task stacks: 14 KB total
- **CPU Usage**: ~30% @ 400 MHz (estimated)
  - Sensor task: ~15%
  - Fusion task: ~10%
  - LED task: <1%

## Technical Details

### Sensor Fusion

The system uses the Madgwick AHRS algorithm for 9-DOF sensor fusion:

1. **Input**: 4x IMU (accel + gyro) + magnetometer
2. **Processing**: Average 4 IMU readings, apply Madgwick filter
3. **Output**: Quaternion and Euler angles (roll, pitch, yaw)

### Calibration

- **Gyroscope**: Zero-rate offset calibration (1000 samples, stationary)
- **Magnetometer**: Hard/soft iron calibration (500 samples, full rotation)

### Thread Safety

- SPI bus protected by mutex
- Lock-free queues for inter-task communication
- Atomic operations where applicable

## Troubleshooting

### Sensor Not Detected

- Check SPI connections
- Verify CS pin assignments
- Ensure power supply is stable

### High Queue Overflow Count

- Reduce sensor rate with `-r` option
- Increase queue sizes in `config.h`
- Check CPU load

### Inaccurate Orientation

- Run calibration with `-c` option
- Check for magnetic interference
- Verify sensor mounting orientation

## References

- [ICM-42688-P Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)
- [BMM150 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/magnetometers/bmm150/)
- [Madgwick AHRS Algorithm](http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
- [NuttX Documentation](https://nuttx.apache.org/docs/latest/)

## License

Apache License 2.0 (following NuttX licensing)
