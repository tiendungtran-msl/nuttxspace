# Telemetry Application - Gửi Dữ Liệu Về Mặt Đất

## 📋 Tổng Quan

`telemetry_app` là ứng dụng chịu trách nhiệm **thu thập dữ liệu từ uORB** và **gửi về Ground Control Station** qua UART. Đây là cầu nối giữa UAV và máy tính trên mặt đất.

### Đặc Điểm

| Thuộc tính | Giá trị | Ghi chú |
|------------|---------|---------|
| Priority | 100 | Thấp nhất - không ảnh hưởng realtime |
| Stack | 4 KB | Đủ cho buffer và state |
| Rate | 100 Hz | 10 ms per packet |
| Baudrate | 921600 | ~92 KB/s throughput |
| Packet size | 212 bytes | Fixed size, CRC protected |

### Vị Trí Trong Hệ Thống

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              UAV System                                     │
│                                                                             │
│  ┌───────────────┐   ┌───────────────┐   ┌───────────────┐                 │
│  │  sensors_app  │   │ estimator_app │   │   state_app   │                 │
│  │  Priority:250 │   │  Priority:240 │   │  Priority:220 │                 │
│  └───────┬───────┘   └───────┬───────┘   └───────┬───────┘                 │
│          │                   │                   │                          │
│          └─────────┬─────────┴───────────┬───────┘                          │
│                    ▼                     ▼                                  │
│          ┌───────────────────────────────────────────┐                     │
│          │                 uORB Bus                   │                     │
│          │  sensor_imu | vehicle_attitude | status   │                     │
│          └────────────────────┬──────────────────────┘                     │
│                               │                                             │
│                               ▼                                             │
│                    ┌─────────────────────┐                                 │
│                    │   TELEMETRY_APP     │ ◀── ĐÂY                         │
│                    │    Priority: 100    │                                 │
│                    │     @ 100 Hz        │                                 │
│                    └──────────┬──────────┘                                 │
│                               │                                             │
│                               ▼                                             │
│                    ┌─────────────────────┐                                 │
│                    │    UART TX (DMA)    │                                 │
│                    │    921600 baud      │                                 │
│                    └──────────┬──────────┘                                 │
└───────────────────────────────┼─────────────────────────────────────────────┘
                                │
                         USB/UART Cable
                                │
                                ▼
                    ┌─────────────────────┐
                    │   Ground Station    │
                    │   (Python Tool)     │
                    └─────────────────────┘
```

---

## 📁 Cấu Trúc Files

```
telemetry_app/
├── Makefile              # Build configuration
├── Kconfig               # Menuconfig options
├── telemetry_main.cpp    # Entry point và main loop
├── telemetry_packet.h    # Packet structure definition
└── telemetry_packet.c    # Packet packing và CRC
```

---

## 📦 Packet Structure (212 bytes)

### Tại Sao 212 Bytes?

- **4 IMU × 28 bytes = 112 bytes** - Raw data từ 4 ICM42688P
- **Mag/Baro/GPS/Attitude/Status = 100 bytes** - Các sensor khác
- **Total = 212 bytes** - Fixed size cho đơn giản

### Layout Chi Tiết

```
┌──────────────────────────────────────────────────────────────────────────┐
│                     TELEMETRY PACKET (212 bytes)                         │
├─────────┬────────┬───────────────────────────────────────────────────────┤
│ Offset  │  Size  │                     Field                             │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│         │        │ HEADER (8 bytes)                                      │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│   0     │   2    │ Magic start: 0x55AA                                   │
│   2     │   2    │ Sequence number (0-65535, wraps)                      │
│   4     │   4    │ Timestamp (µs since boot)                              │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│         │        │ IMU DATA (28 bytes × 4 = 112 bytes)                   │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│   8     │   28   │ IMU 0: Gx,Gy,Gz,Ax,Ay,Az,Temp (7 floats)              │
│  36     │   28   │ IMU 1: Gx,Gy,Gz,Ax,Ay,Az,Temp (7 floats)              │
│  64     │   28   │ IMU 2: Gx,Gy,Gz,Ax,Ay,Az,Temp (7 floats)              │
│  92     │   28   │ IMU 3: Gx,Gy,Gz,Ax,Ay,Az,Temp (7 floats)              │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│         │        │ OTHER SENSORS                                         │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│ 120     │   12   │ Magnetometer: Mx,My,Mz (Gauss)                        │
│ 132     │    8   │ Barometer: Pressure(Pa), Altitude(m)                  │
│ 140     │   24   │ GPS: lat,lon,alt,speed,heading,fix,sats,hdop,vdop     │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│         │        │ ATTITUDE (32 bytes)                                   │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│ 164     │   16   │ Quaternion: W,X,Y,Z (normalized)                      │
│ 180     │   12   │ Euler: Roll,Pitch,Yaw (radians)                       │
│ 192     │    4   │ EKF innovation variance                               │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│         │        │ STATUS (12 bytes)                                     │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│ 196     │    1   │ Health level (0=GOOD, 1=WARN, 2=CRIT, 3=FAIL)         │
│ 197     │    1   │ Healthy IMU bitmask (bit0=IMU0, bit1=IMU1,...)        │
│ 198     │    1   │ Sensor status flags                                   │
│ 199     │    1   │ Reserved (padding)                                    │
│ 200     │    2   │ CPU load (0-1000 = 0-100.0%)                          │
│ 202     │    2   │ Battery voltage (mV)                                  │
│ 204     │    4   │ Loop count                                            │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│         │        │ FOOTER (4 bytes)                                      │
├─────────┼────────┼───────────────────────────────────────────────────────┤
│ 208     │    2   │ CRC16-CCITT                                           │
│ 210     │    2   │ Magic end: 0xAA55                                     │
└─────────┴────────┴───────────────────────────────────────────────────────┘
```

### Tại Sao Gửi 4 IMU Raw?

1. **Debug:** Xem từng IMU hoạt động đúng không
2. **Fusion analysis:** So sánh 4 IMU, phát hiện outlier
3. **Calibration:** Thu thập data để calibrate offline
4. **Redundancy check:** Verify voting algorithm

---

## 🔄 Main Loop

### Flow

```cpp
int telemetry_main(int argc, char *argv[])
{
    // 1. Subscribe uORB topics
    int imu_sub[4];
    for (int i = 0; i < 4; i++) {
        imu_sub[i] = orb_subscribe_multi(ORB_ID(sensor_imu), i);
    }
    int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    int status_sub = orb_subscribe(ORB_ID(system_status));
    
    // 2. Open UART
    int uart_fd = open("/dev/ttyS1", O_WRONLY | O_NONBLOCK);
    configure_uart(uart_fd, 921600);
    
    // 3. Main loop @ 100 Hz
    while (!should_exit) {
        uint64_t start = hrt_absolute_time();
        
        // Collect data from uORB
        collect_sensor_data();
        
        // Pack into packet
        pack_telemetry_packet(&packet);
        
        // Calculate CRC
        packet.crc = calculate_crc16(&packet);
        
        // Send via UART
        write(uart_fd, &packet, sizeof(packet));
        
        // Sleep until next period
        uint64_t elapsed = hrt_absolute_time() - start;
        if (elapsed < 10000) {  // 10 ms period
            usleep(10000 - elapsed);
        }
    }
}
```

### Timing Budget

```
┌────────────────────────────────────────────────────────────────────────────┐
│                         TIMING (10 ms period)                              │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│  ├── 0.0 ms: uORB subscribe & copy (~50 µs × 6 topics = 0.3 ms)           │
│  ├── 0.3 ms: Pack packet (~20 µs)                                         │
│  ├── 0.4 ms: Calculate CRC (~10 µs)                                       │
│  ├── 0.4 ms: write() to UART buffer (~5 µs)                               │
│  ├── 0.4 - 2.7 ms: DMA transfer (212 bytes @ 921600 = ~2.3 ms)            │
│  ├── 2.7 ms: Task có thể sleep                                            │
│  └── 10.0 ms: Next iteration                                              │
│                                                                            │
│  CPU usage: ~4% (0.4 ms / 10 ms)                                          │
│  UART usage: ~23% (2.3 ms / 10 ms)                                        │
│                                                                            │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 🔧 CRC16-CCITT

### Implementation

```c
/**
 * CRC16-CCITT với:
 * - Initial value: 0xFFFF
 * - Polynomial: 0x1021
 * - Input/Output: không reflect
 */
uint16_t telemetry_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}
```

### Usage

```c
// CRC tính trên bytes 0-207 (không bao gồm CRC field và end magic)
packet.crc = telemetry_crc16((uint8_t*)&packet, 
                             offsetof(struct telemetry_packet_s, crc));
```

---

## 📝 telemetry_packet.h

### Struct Definition

```c
#pragma pack(push, 1)  // Đảm bảo không padding

struct telemetry_imu_data_s {
    float gyro_x;       // rad/s
    float gyro_y;
    float gyro_z;
    float accel_x;      // m/s²
    float accel_y;
    float accel_z;
    float temperature;  // °C
};

struct telemetry_packet_s {
    // Header (8 bytes)
    uint16_t magic_start;   // 0x55AA
    uint16_t sequence;
    uint32_t timestamp_us;
    
    // IMU data (112 bytes)
    struct telemetry_imu_data_s imu[4];
    
    // Mag (12 bytes)
    float mag_x, mag_y, mag_z;
    
    // Baro (8 bytes)
    float pressure;
    float baro_alt;
    
    // GPS (24 bytes)
    int32_t latitude;       // deg × 1e7
    int32_t longitude;
    int32_t altitude_msl;   // mm
    uint32_t ground_speed;  // cm/s
    int16_t heading;        // deg × 100
    uint8_t fix_type;
    uint8_t satellites;
    uint16_t hdop;
    uint16_t vdop;
    
    // Attitude (32 bytes)
    float quat_w, quat_x, quat_y, quat_z;
    float roll, pitch, yaw;
    float innovation_var;
    
    // Status (12 bytes)
    uint8_t health_level;
    uint8_t healthy_imus;
    uint8_t sensor_flags;
    uint8_t reserved;
    uint16_t cpu_load;
    uint16_t battery_mv;
    uint32_t loop_count;
    
    // Footer (4 bytes)
    uint16_t crc;
    uint16_t magic_end;     // 0xAA55
};

#pragma pack(pop)

// Verify size at compile time
_Static_assert(sizeof(struct telemetry_packet_s) == 212,
               "Packet size must be 212 bytes");
```

---

## 🎛️ Kconfig Options

```kconfig
config UAV_TELEMETRY
    bool "Enable UAV Telemetry Application"
    default y
    ---help---
        Enable the telemetry application for ground station communication.

config UAV_TELEMETRY_PRIORITY
    int "Task priority"
    default 100
    depends on UAV_TELEMETRY

config UAV_TELEMETRY_STACKSIZE
    int "Stack size"
    default 4096
    depends on UAV_TELEMETRY

config UAV_TELEMETRY_RATE_HZ
    int "Telemetry rate (Hz)"
    default 100
    depends on UAV_TELEMETRY

config UAV_TELEMETRY_UART_DEV
    string "UART device"
    default "/dev/ttyS1"
    depends on UAV_TELEMETRY

config UAV_TELEMETRY_BAUDRATE
    int "UART baudrate"
    default 921600
    depends on UAV_TELEMETRY
```

---

## 🖥️ Sử Dụng

### Lệnh NSH

```bash
# Khởi động telemetry
nsh> telemetry start

# Xem trạng thái
nsh> telemetry status
Telemetry Status:
  Running: YES
  UART: /dev/ttyS1 @ 921600
  Rate: 100 Hz
  Packets sent: 12345
  Errors: 0
  Last send: 5 ms ago

# Dừng telemetry
nsh> telemetry stop
```

### Ground Station Side

```bash
# Chạy GCS tool
cd /home/msi-leibniz/nuttxspace/tools
python3 run_gcs_fast.py

# Hoặc xem raw data
python3 dump_raw.py /dev/ttyUSB0 921600
```

---

## 🐛 Debug

### 1. Không Nhận Được Data

```bash
# Kiểm tra UART có output không
nsh> cat /dev/ttyS1
# Phải thấy binary data

# Kiểm tra baud rate
# Cả hai bên phải cùng 921600
```

### 2. CRC Errors Cao

```bash
# Có thể do:
# - Cable kém chất lượng
# - Baudrate không khớp
# - EMI từ motor

# Thử giảm baudrate
CONFIG_UAV_TELEMETRY_BAUDRATE=115200
```

### 3. Rate Thấp

```bash
# Kiểm tra task không bị starve
nsh> ps
# telemetry phải có CPU time

# Kiểm tra UART buffer đầy
# Tăng buffer size nếu cần
```

---

## 🔗 Liên Kết

- **Ground Station Tool:** `/tools/` → `run_gcs_fast.py`
- **Protocol Docs:** `/apps/uav/TELEMETRY_ARCHITECTURE.md`
- **Python Decoder:** `/tools/uav_gcs/protocol/packet.py`
