# drivers/spi - SPI Device Framework

## 📋 Tổng Quan

Module `spi` cung cấp **base class cho SPI devices** trong NuttX:
- **Device:** Base class với thread-safe transfers
- **Config:** Configuration structure
- **Helpers:** Register read/write utilities

---

## 📁 Files

```
spi/
├── Makefile
├── spi_device.hpp    # Base class definition
├── spi_device.cpp    # Implementation
└── spi_types.hpp     # Types và configuration
```

---

## 🎯 Thiết Kế

### Class Hierarchy

```
┌─────────────────────────────────────────────────┐
│                spi::Device                       │
│  (Base class cho tất cả SPI devices)            │
├─────────────────────────────────────────────────┤
│  - Thread-safe transfers                         │
│  - Frequency control                             │
│  - Register read/write                           │
│  - DeviceId management                           │
└──────────────────────┬──────────────────────────┘
                       │
          ┌────────────┼────────────┐
          ▼            ▼            ▼
     ┌─────────┐  ┌─────────┐  ┌─────────┐
     │ICM42688P│  │ BMP280  │  │ W25Q128 │
     │ (IMU)   │  │ (Baro)  │  │ (Flash) │
     └─────────┘  └─────────┘  └─────────┘
```

### Thread Safety

```
Multiple tasks accessing same SPI bus:

     Task A                Task B                Task C
        │                     │                     │
        ▼                     ▼                     ▼
   ┌─────────┐           ┌─────────┐           ┌─────────┐
   │ Device1 │           │ Device2 │           │ Device3 │
   │(IMU CS0)│           │(IMU CS1)│           │(Baro)   │
   └────┬────┘           └────┬────┘           └────┬────┘
        │                     │                     │
        └─────────────┬───────┴─────────────────────┘
                      │
                ┌─────▼─────┐
                │   MUTEX   │
                │  (lock)   │
                └─────┬─────┘
                      │
                ┌─────▼─────┐
                │   SPI4    │
                │ Hardware  │
                └───────────┘
```

---

## 📐 Configuration

### spi_types.hpp

```cpp
namespace drivers {
namespace spi {

/**
 * @brief Lock modes cho thread safety
 */
enum class LockMode : uint8_t {
    NONE,       // Không lock (single-threaded)
    CRITICAL,   // Disable interrupts
    MUTEX       // NuttX mutex (default)
};

/**
 * @brief SPI configuration
 */
struct Config {
    uint8_t bus;            // SPI bus number (1, 2, 3, ...)
    uint8_t cs;             // Chip select index
    uint32_t frequency;     // Clock frequency (Hz)
    uint8_t mode;           // SPI mode (0, 1, 2, 3)
    uint8_t bits;           // Bits per word (8 or 16)
    LockMode lock_mode;     // Thread safety mode
};

} // namespace spi
} // namespace drivers
```

### SPI Modes

```
Mode 0: CPOL=0, CPHA=0 (most common)
        ─┐    ┌──┐    ┌──┐    ┌──┐    ┌──
Clock    └────┘  └────┘  └────┘  └────┘
        ↑    ↑    ↑    ↑    ↑    ↑    ↑
      Sample at rising edge

Mode 1: CPOL=0, CPHA=1
Mode 2: CPOL=1, CPHA=0  
Mode 3: CPOL=1, CPHA=1
```

---

## 🔧 API

### Constructor

```cpp
#include <uav/drivers/spi/spi_device.hpp>

using namespace drivers::spi;

// Configuration
Config config;
config.bus = 4;                  // SPI4
config.cs = 0;                   // CS0
config.frequency = 8000000;      // 8 MHz
config.mode = 0;                 // Mode 0 (CPOL=0, CPHA=0)
config.bits = 8;                 // 8-bit transfers
config.lock_mode = LockMode::MUTEX;

// Create device với device type
Device dev(config, DeviceType::ICM42688P);

// Initialize
int ret = dev.init();
if (ret != 0) {
    printf("SPI init failed: %d\n", ret);
}
```

### Raw Transfers

```cpp
// 8-bit transfer
uint8_t tx[] = {0x75 | 0x80, 0x00};  // Read WHO_AM_I
uint8_t rx[2];
dev.transfer(tx, rx, 2);
// rx[1] contains register value

// Send only (ignore received)
uint8_t cmd[] = {0x4E, 0x0F};  // Write to PWR_MGMT0
dev.transfer(cmd, nullptr, 2);

// Receive only (send zeros)
uint8_t data[14];
dev.transfer(nullptr, data, 14);
```

### Register Operations

```cpp
// Read single register
uint8_t value;
dev.read_reg(0x75, &value);  // WHO_AM_I

// Write single register
dev.write_reg(0x4E, 0x0F);   // PWR_MGMT0

// Read multiple registers (burst)
uint8_t buf[14];
dev.read_regs(0x1D, buf, 14);  // Read from TEMP_DATA1
```

### Frequency Control

```cpp
// Giảm frequency cho init (safer)
dev.set_frequency(1000000);  // 1 MHz

// Sau init, tăng frequency
dev.set_frequency(8000000);  // 8 MHz

// Get current frequency
uint32_t freq = dev.get_frequency();
```

---

## 🔒 Lock Modes

### NONE

```cpp
config.lock_mode = LockMode::NONE;

// Không có bảo vệ
// Chỉ dùng khi chắc chắn single-threaded
// Fastest nhưng nguy hiểm
```

### CRITICAL (Disable IRQ)

```cpp
config.lock_mode = LockMode::CRITICAL;

// Disable interrupts trong transfer
// Latency thấp nhất
// Nhưng block tất cả interrupts!
// Chỉ dùng cho transfers rất ngắn
```

### MUTEX (Recommended)

```cpp
config.lock_mode = LockMode::MUTEX;

// NuttX mutex protect
// Thread-safe
// Allows other interrupts
// Slight latency overhead
```

---

## 📊 Timing

### SPI Transfer Overhead

```
NuttX SPI call overhead: ~5-10 µs
Lock/unlock: ~1 µs (mutex)
             ~0.5 µs (critical section)

Example 14-byte read @ 8 MHz:
  Data: 14 × 8 bits / 8 MHz = 14 µs
  Overhead: ~10 µs
  Total: ~24 µs
```

### Optimization Tips

```cpp
// ❌ Bad: Multiple small transfers
for (int i = 0; i < 14; i++) {
    dev.read_reg(0x1D + i, &buf[i]);  // 14 × 24 µs = 336 µs
}

// ✅ Good: Single burst transfer
dev.read_regs(0x1D, buf, 14);  // 1 × 38 µs = 38 µs
```

---

## 💡 Best Practices

### 1. Inherit từ Device

```cpp
class ICM42688P : private spi::Device {
public:
    ICM42688P(const spi::Config& config)
        : Device(config, DeviceType::ICM42688P)
    {}
    
    int init() {
        int ret = Device::init();
        if (ret != 0) return ret;
        
        // Chip-specific init
        return configure_sensor();
    }
    
    int read_data(ImuData* data) {
        uint8_t buf[14];
        read_regs(FIRST_DATA_REG, buf, 14);
        parse_data(buf, data);
        return 0;
    }
};
```

### 2. Error Handling

```cpp
int safe_transfer(const uint8_t* tx, uint8_t* rx, size_t len)
{
    if (!is_initialized()) {
        return -ENXIO;
    }
    
    int ret = transfer(tx, rx, len);
    if (ret != 0) {
        _error_count++;
        if (_error_count > MAX_ERRORS) {
            mark_device_failed();
        }
    }
    return ret;
}
```

### 3. Dynamic Frequency

```cpp
// Slower frequency for initialization
void init_device() {
    set_frequency(1000000);  // 1 MHz - safe
    
    // Read WHO_AM_I
    verify_chip_id();
    
    // Configure
    configure_registers();
    
    // Switch to fast mode
    set_frequency(8000000);  // 8 MHz - fast
}
```

---

## 📐 Memory Layout

```cpp
class Device {
    struct spi_dev_s* _dev;      // 4 bytes (pointer)
    Config _config;               // 12 bytes
    DeviceId _device_id;         // 4 bytes
    sem_t _lock;                 // 8 bytes (NuttX semaphore)
    // Total: ~28 bytes per instance
};
```

---

## 🔗 Liên Kết

- **Usage:** `/apps/uav/drivers/imu/`
- **Device ID:** `/apps/uav/lib/drivers_framework/`
- **NuttX SPI:** `nuttx/include/nuttx/spi/spi.h`
