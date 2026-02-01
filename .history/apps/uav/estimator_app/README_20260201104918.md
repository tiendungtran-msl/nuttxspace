# Estimator Application

## Tổng Quan

`estimator_app` là app ước lượng trạng thái (state estimation), chạy EKF2 và
attitude filter để tính toán tư thế và vị trí của UAV.

## Vai Trò

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          ESTIMATOR_APP                                      │
│                        Priority: 240                                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  INPUT (Subscribe từ uORB):                                                 │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │ sensor_imu  │ │ sensor_mag  │ │ sensor_baro │ │ sensor_gps  │           │
│  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘ └──────┬──────┘           │
│         │               │               │               │                  │
│         ▼               ▼               ▼               ▼                  │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │                         EKF2 CORE                                  │    │
│  │  ┌─────────────────┐    ┌─────────────────┐                       │    │
│  │  │   PREDICTION    │    │     FUSION      │                       │    │
│  │  │  (IMU @ 250Hz)  │───►│  (GPS/Mag/Baro) │                       │    │
│  │  └─────────────────┘    └─────────────────┘                       │    │
│  │                                                                    │    │
│  │  State Vector [16]:                                               │    │
│  │  [quat(4), vel(3), pos(3), gyro_bias(3), accel_bias(3)]          │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│         │               │               │                                  │
│         ▼               ▼               ▼                                  │
│  OUTPUT (Publish lên uORB):                                                │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐                          │
│  │vehicle_att  │ │ vehicle_pos │ │ ekf2_status │                          │
│  └─────────────┘ └─────────────┘ └─────────────┘                          │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Thuật Toán

### EKF2 (Extended Kalman Filter 2)

**State Vector (16 phần tử):**
- `quat[4]`: Quaternion (body → NED)
- `vel[3]`: Velocity NED [m/s]
- `pos[3]`: Position NED [m]
- `gyro_bias[3]`: Gyro bias [rad/s]
- `accel_bias[3]`: Accel bias [m/s²]

**Prediction Step (chạy mỗi IMU sample):**
```
1. Bù bias: gyro_corrected = gyro - gyro_bias
2. Integrate quaternion: q = q ⊗ Δq(gyro_corrected * dt)
3. Xoay accel về NED: accel_ned = R(q) * accel_corrected
4. Integrate velocity: vel += (accel_ned + gravity) * dt
5. Integrate position: pos += vel * dt
6. Propagate covariance: P = F*P*F' + Q
```

**Fusion Steps (chạy khi có sensor data):**
- GPS velocity fusion
- GPS position fusion
- Barometer altitude fusion
- Magnetometer heading fusion

### Sensor Selector

Khi có multi-IMU, cần chọn IMU tốt nhất:
- Health check (data rate, variance)
- Voting algorithm
- Failover khi IMU primary lỗi

## Event-Driven Design

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         poll() BASED LOOP                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  while (!exit) {                                                            │
│      ret = poll(fds, n, timeout);                                           │
│                                                                             │
│      if (fds[IMU].revents) {                                                │
│          orb_copy(sensor_imu, ...);                                         │
│          ekf.predict(imu);         // Chạy prediction                       │
│      }                                                                      │
│                                                                             │
│      if (fds[GPS].revents) {                                                │
│          orb_copy(sensor_gps, ...);                                         │
│          ekf.setGpsData(gps);      // Queue cho fusion                      │
│      }                                                                      │
│                                                                             │
│      if (ekf.update()) {           // Chạy fusion + output                  │
│          publish_attitude();                                                │
│          publish_position();                                                │
│      }                                                                      │
│  }                                                                          │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Tại Sao Event-Driven?

1. **Thấp latency**: Xử lý ngay khi có data, không đợi timer
2. **Tiết kiệm CPU**: Sleep khi không có data (vs busy polling)
3. **Tự động rate matching**: Chạy theo rate của sensors

## File Structure

```
estimator_app/
├── README.md               # File này
├── Makefile
├── Kconfig
├── estimator_main.cpp      # Entry point + main loop
├── sensor_selector.hpp     # Multi-IMU selection
└── sensor_selector.cpp
```

## Khởi Động

```bash
nsh> estimator start
[estimator] Waiting for IMU...
[estimator] IMU detected, starting EKF2
[estimator] Tilt aligned
[estimator] Yaw aligned (mag)
[estimator] GPS fusion started

nsh> estimator status
[estimator] Running
  Attitude valid: YES
  Position valid: YES
  GPS fused: YES
  Baro fused: YES
```

## Dependencies

- `sensors_app` phải chạy trước
- `uorb` library
- `lib/ekf2` (EKF implementation)
