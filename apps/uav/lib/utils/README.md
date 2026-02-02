# lib/utils - Utility Libraries

## 📋 Tổng Quan

Module `utils` chứa các **utility classes** và **macros** được dùng chung trong toàn bộ hệ thống UAV.

---

## 📁 Files

```
utils/
├── Makefile
├── ringbuf.hpp      # Lock-free ring buffer
├── dma_buffer.hpp   # DMA double buffer
├── critical.hpp     # Critical section helpers
└── debug.hpp        # Debug utilities
```

---

## 🔄 ringbuf.hpp - Lock-Free Ring Buffer

### Mục Đích

Truyền dữ liệu an toàn giữa **producer** và **consumer** mà không cần mutex. Thích hợp cho:
- ISR → Task communication
- DMA callback → Processing task
- High-rate data streaming

### Đặc Điểm

| Feature | Mô Tả |
|---------|-------|
| Pattern | SPSC (Single Producer Single Consumer) |
| Lock-free | Không cần mutex/semaphore |
| Fixed size | Compile-time size, power of 2 |
| Memory barriers | ARM DMB instruction |

### API

```cpp
#include <uav/lib/utils/ringbuf.hpp>

using namespace uav::utils;

// Tạo buffer: <Type, Size>
// Size PHẢI là power of 2 (2, 4, 8, 16, 32, ...)
RingBuffer<sensor_imu_s, 16> imu_ring;

// ─── Producer (ISR/DMA) ───
bool success = imu_ring.push(sample);
if (!success) {
    // Buffer đầy, sample bị drop
    overflow_count++;
}

// ─── Consumer (Task) ───
sensor_imu_s sample;
if (imu_ring.pop(&sample)) {
    // Xử lý sample
}

// Hoặc pop nhiều cùng lúc
sensor_imu_s batch[8];
int count = imu_ring.pop_batch(batch, 8);

// Check status
bool empty = imu_ring.empty();
bool full = imu_ring.full();
size_t available = imu_ring.available();
```

### Sơ Đồ Hoạt Động

```
                    Ring Buffer [16]
     ┌────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┐
     │ 0  │ 1  │ 2  │ 3  │ 4  │ 5  │ 6  │ 7  │ 8  │ 9  │ 10 │ 11 │ 12 │ 13 │ 14 │ 15 │
     └────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┘
               ▲                             ▲
               │                             │
             tail                          head
           (consumer)                    (producer)
           
     Available = head - tail = 4 samples
```

### Memory Barrier

```cpp
// ARM Cortex-M: DMB instruction
// Đảm bảo thứ tự memory operations
#define UAV_MEMORY_BARRIER() __asm__ __volatile__("dmb" ::: "memory")

// Dùng khi:
// - Producer: sau khi write data, trước khi update head
// - Consumer: sau khi read head, trước khi read data
```

### Example: IMU DMA

```cpp
// Global buffer
static RingBuffer<sensor_imu_s, 32> g_imu_ring;

// DMA complete callback (ISR context)
void imu_dma_complete_isr(void)
{
    sensor_imu_s sample;
    sample.timestamp = hrt_absolute_time();
    sample.gyro[0] = raw_buffer.gyro_x * GYRO_SCALE;
    // ... fill sample
    
    g_imu_ring.push(sample);  // Lock-free, ISR-safe
}

// Processing task
void imu_task_main(void)
{
    while (!should_exit) {
        sensor_imu_s samples[16];
        int count = g_imu_ring.pop_batch(samples, 16);
        
        for (int i = 0; i < count; i++) {
            process_imu(samples[i]);
        }
        
        usleep(1000);  // 1 kHz check
    }
}
```

---

## 🔀 dma_buffer.hpp - DMA Double Buffer

### Mục Đích

Quản lý **double buffering** cho DMA transfers:
- DMA writes to **back buffer**
- CPU reads from **front buffer**
- **Swap** khi DMA complete

### API

```cpp
#include <uav/lib/utils/dma_buffer.hpp>

using namespace uav::utils;

// Tạo double buffer
DmaDoubleBuffer<ImuRawData> dma_buf;

// ─── Setup DMA ───
ImuRawData* dma_target = dma_buf.get_dma_target();
configure_dma(dma_target, sizeof(ImuRawData));
start_dma();

// ─── DMA Complete ISR ───
void dma_complete_isr(void)
{
    dma_buf.swap();  // Swap buffers
    
    // Restart DMA to new back buffer
    ImuRawData* next = dma_buf.get_dma_target();
    restart_dma(next);
}

// ─── Processing Task ───
const ImuRawData* data = dma_buf.get_read_buffer();
// Safe to read - DMA is writing to other buffer
```

### Sơ Đồ

```
Time T0:                           Time T1 (after swap):
┌────────────────┐                 ┌────────────────┐
│   Buffer A     │ ◀── CPU reads   │   Buffer A     │ ◀── DMA writes
│  (Front/Read)  │                 │  (Back/Write)  │
└────────────────┘                 └────────────────┘

┌────────────────┐                 ┌────────────────┐
│   Buffer B     │ ◀── DMA writes  │   Buffer B     │ ◀── CPU reads
│  (Back/Write)  │                 │  (Front/Read)  │
└────────────────┘                 └────────────────┘
```

---

## 🔒 critical.hpp - Critical Sections

### Mục Đích

Helpers cho **critical sections** (disable interrupts):
- Atomic operations
- Register access
- Short sequences

### API

```cpp
#include <uav/lib/utils/critical.hpp>

using namespace uav::utils;

// RAII-style critical section
{
    CriticalSection cs;  // Disable IRQ
    
    // Atomic operations here
    value = shared_variable;
    shared_variable = new_value;
    
}  // IRQ re-enabled on destructor

// Hoặc manual
irqstate_t flags = enter_critical_section();
// ... critical code
leave_critical_section(flags);
```

### Lưu Ý

```cpp
// ⚠️ CRITICAL: Giữ critical section NGẮN
// Target: < 1 µs
// Max: < 10 µs

// ❌ Sai - quá lâu
{
    CriticalSection cs;
    memcpy(dst, src, 1000);  // Too long!
}

// ✅ Đúng - chỉ access pointer
{
    CriticalSection cs;
    data = shared_ptr;
}
memcpy(dst, data, 1000);  // Outside critical
```

---

## 🐛 debug.hpp - Debug Utilities

### Mục Đích

Macros và helpers cho debugging:
- Compile-time asserts
- Runtime assertions
- Debug logging

### API

```cpp
#include <uav/lib/utils/debug.hpp>

// Compile-time assert
UAV_STATIC_ASSERT(sizeof(packet) == 212, "Wrong packet size");

// Runtime assert (debug build only)
UAV_ASSERT(ptr != nullptr);

// Debug log (disabled in release)
UAV_DEBUG("IMU rate: %d Hz", rate);

// Error log (always enabled)
UAV_ERROR("Failed to init IMU: %d", ret);

// Performance marker
UAV_PERF_BEGIN("ekf_predict");
run_ekf_predict();
UAV_PERF_END("ekf_predict");
```

### Build Configurations

```cpp
#ifdef CONFIG_UAV_DEBUG
    // Debug build: assertions enabled
    #define UAV_ASSERT(x) if(!(x)) panic()
    #define UAV_DEBUG(...) syslog(LOG_DEBUG, __VA_ARGS__)
#else
    // Release build: assertions disabled
    #define UAV_ASSERT(x) ((void)0)
    #define UAV_DEBUG(...) ((void)0)
#endif
```

---

## 📊 Memory Usage

| Class | Size |
|-------|------|
| RingBuffer<T, 16> | 16 × sizeof(T) + 16 bytes overhead |
| DmaDoubleBuffer<T> | 2 × sizeof(T) + 8 bytes |
| CriticalSection | 4 bytes (stack) |

---

## 🔗 Liên Kết

- **Usage:** `/apps/uav/sensors_app/`, `/apps/uav/estimator_app/`
- **Architecture:** `/apps/uav/ARCHITECTURE.md`
