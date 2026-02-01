# uORB - Hệ Thống Message Bus

## Tổng Quan

uORB (micro Object Request Broker) là hệ thống **publish/subscribe** cho phép
các app giao tiếp với nhau mà không cần gọi hàm trực tiếp.

## Tại Sao Cần uORB?

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         VẤN ĐỀ KHI KHÔNG CÓ uORB                            │
├─────────────────────────────────────────────────────────────────────────────┤
│ ❌ Tight coupling: sensors.getData() gọi trực tiếp từ estimator            │
│ ❌ Khó test: Không thể test estimator mà không có sensors thật             │
│ ❌ Khó mở rộng: Thêm logger phải sửa code sensors                          │
│ ❌ Race condition: Nhiều thread đọc/ghi cùng data                          │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                         GIẢI PHÁP VỚI uORB                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│ ✅ Loose coupling: sensors publish, estimator subscribe                     │
│ ✅ Dễ test: Mock topic data cho estimator                                   │
│ ✅ Dễ mở rộng: Logger subscribe không cần sửa sensors                       │
│ ✅ Thread-safe: Mutex + sequence number bảo vệ data                         │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Kiến Trúc

```
┌─────────────┐                              ┌─────────────┐
│  Publisher  │                              │ Subscriber  │
│ (sensors)   │                              │ (estimator) │
└──────┬──────┘                              └──────┬──────┘
       │                                            │
       │ orb_publish()                              │ orb_copy()
       ▼                                            ▼
┌─────────────────────────────────────────────────────────────┐
│                        uORB Topic                           │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                  Ring Buffer [8]                     │   │
│  │  [msg0][msg1][msg2][msg3][msg4][msg5][msg6][msg7]   │   │
│  └─────────────────────────────────────────────────────┘   │
│  sequence: 1234    mutex: locked/unlocked                  │
└─────────────────────────────────────────────────────────────┘
```

## API Chính

```cpp
// Publisher
orb_advert_t pub = orb_advertise(ORB_ID(sensor_imu), &initial_data);
orb_publish(ORB_ID(sensor_imu), pub, &new_data);
orb_unadvertise(pub);

// Subscriber  
int sub = orb_subscribe(ORB_ID(sensor_imu));
bool updated;
orb_check(sub, &updated);
if (updated) {
    sensor_imu_s data;
    orb_copy(ORB_ID(sensor_imu), sub, &data);
}
orb_unsubscribe(sub);
```

## Định Nghĩa Topic

Mỗi topic được định nghĩa trong `topics/` với:
1. **Message struct**: Dữ liệu POD (Plain Old Data)
2. **ORB_DECLARE**: Macro đăng ký topic

```cpp
// topics/sensor_imu.hpp
struct sensor_imu_s {
    uint64_t timestamp_us;
    float accel[3];
    float gyro[3];
    float temperature;
    uint8_t instance;
};

ORB_DECLARE(sensor_imu);
```

## Thread Safety

- Mỗi topic có một **mutex** bảo vệ
- **Sequence number** giúp detect cập nhật mới
- **Ring buffer** cho phép miss 1-2 sample không mất data
- **Copy semantics**: Subscriber copy data, không reference

## File Structure

```
uorb/
├── README.md           # File này
├── Makefile
├── Kconfig
├── uorb.hpp            # API declarations
├── uorb.cpp            # Implementation
├── orb_defines.hpp     # Macros và constants
└── topics/
    ├── sensor_imu.hpp
    ├── sensor_mag.hpp
    ├── sensor_baro.hpp
    ├── sensor_gps.hpp
    ├── vehicle_attitude.hpp
    ├── vehicle_local_position.hpp
    ├── vehicle_state.hpp
    └── ekf2_status.hpp
```

## Lưu Ý Quan Trọng

1. **Không dùng trong ISR**: API có mutex, không safe trong interrupt
2. **Copy nhanh**: Message nên nhỏ (< 256 bytes) để copy nhanh
3. **Timestamp bắt buộc**: Mọi message phải có `timestamp_us`
4. **POD only**: Không pointer, không virtual function trong message
