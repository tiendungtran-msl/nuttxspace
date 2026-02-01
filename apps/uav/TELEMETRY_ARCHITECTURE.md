# UAV Telemetry System Architecture

## Tài liệu thiết kế kỹ thuật - Ground Station Tool

**Phiên bản**: 1.0  
**Ngày**: 2026-02-01  
**Tác giả**: UAV Avionics Team

---

## 1. KIẾN TRÚC TỔNG THỂ

### 1.1 Sơ đồ End-to-End

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              UAV (STM32H743 + NuttX)                        │
│  ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐                     │
│  │ IMU x4  │   │  Baro   │   │   Mag   │   │   GPS   │                     │
│  └────┬────┘   └────┬────┘   └────┬────┘   └────┬────┘                     │
│       │             │             │             │                           │
│       └──────────┬──┴─────────────┴──────┬──────┘                           │
│                  ▼                       ▼                                  │
│           ┌─────────────┐         ┌─────────────┐                          │
│           │ sensors_app │         │   EKF Task  │                          │
│           │   @ 1kHz    │         │   @ 250Hz   │                          │
│           └──────┬──────┘         └──────┬──────┘                          │
│                  │                       │                                  │
│                  ▼                       ▼                                  │
│           ┌──────────────────────────────────────┐                         │
│           │              uORB Bus                 │                         │
│           │  sensor_combined | vehicle_attitude  │                         │
│           │  sensor_baro | sensor_mag | gps_pos  │                         │
│           └───────────────────┬──────────────────┘                         │
│                               │                                             │
│                               ▼                                             │
│                    ┌─────────────────────┐                                 │
│                    │   telemetry_app     │                                 │
│                    │      @ 100Hz        │                                 │
│                    │  Priority: LOW      │                                 │
│                    └──────────┬──────────┘                                 │
│                               │                                             │
│                               ▼                                             │
│                    ┌─────────────────────┐                                 │
│                    │   UART TX (DMA)     │                                 │
│                    │   921600 baud       │                                 │
│                    └──────────┬──────────┘                                 │
└───────────────────────────────┼─────────────────────────────────────────────┘
                                │
                         ┌──────┴──────┐
                         │  UART/USB   │
                         │   Cable     │
                         └──────┬──────┘
                                │
┌───────────────────────────────┼─────────────────────────────────────────────┐
│                               ▼                    PC (Python Tool)         │
│                    ┌─────────────────────┐                                 │
│                    │  Serial Port        │                                 │
│                    │  /dev/ttyUSB0       │                                 │
│                    └──────────┬──────────┘                                 │
│                               │                                             │
│                               ▼                                             │
│                    ┌─────────────────────┐                                 │
│                    │   Receiver Thread   │                                 │
│                    │   (Non-blocking)    │                                 │
│                    └──────────┬──────────┘                                 │
│                               │                                             │
│                               ▼                                             │
│                    ┌─────────────────────┐                                 │
│                    │   Packet Decoder    │                                 │
│                    │   + CRC Validate    │                                 │
│                    └──────────┬──────────┘                                 │
│                               │                                             │
│                               ▼                                             │
│                    ┌─────────────────────┐                                 │
│                    │    Data Buffer      │                                 │
│                    │   (Thread-safe)     │                                 │
│                    └──────────┬──────────┘                                 │
│                               │                                             │
│          ┌────────────────────┼────────────────────┐                       │
│          ▼                    ▼                    ▼                       │
│  ┌───────────────┐   ┌───────────────┐   ┌───────────────┐                │
│  │  3D Attitude  │   │  Plot Graphs  │   │  Data Table   │                │
│  │  Visualizer   │   │  (PyQtGraph)  │   │   (Realtime)  │                │
│  └───────────────┘   └───────────────┘   └───────────────┘                │
│                                                                             │
│                    ┌─────────────────────┐                                 │
│                    │   Logger (SQLite)   │                                 │
│                    └─────────────────────┘                                 │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Timing Budget

```
┌────────────────────────────────────────────────────────────────────────────┐
│                         TIMING DIAGRAM (10ms period)                       │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│  MCU Side (100 Hz telemetry):                                              │
│  ├── t=0.0ms:  uORB subscribe & copy data (~50µs)                         │
│  ├── t=0.1ms:  Pack binary packet (~20µs)                                 │
│  ├── t=0.2ms:  UART DMA start (~5µs)                                      │
│  ├── t=0.2-1.5ms: DMA transfer (128 bytes @ 921600 = ~1.4ms)              │
│  └── t=1.5ms:  DMA complete, task sleep until next period                 │
│                                                                            │
│  PC Side:                                                                  │
│  ├── Receiver thread: Continuous read, ~100µs latency                     │
│  ├── Decode: ~50µs per packet                                             │
│  ├── GUI update: 16.7ms period (60 FPS)                                   │
│  └── Plot update: 33.3ms period (30 FPS)                                  │
│                                                                            │
│  End-to-end latency: ~5-10ms (UART + USB + OS scheduling)                 │
│                                                                            │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. GIAO THỨC BINARY PACKET

### 2.1 Packet Structure

```
┌────────────────────────────────────────────────────────────────────────────┐
│                        TELEMETRY PACKET (128 bytes)                        │
├────────┬────────┬────────────────────────────────────────────────┬─────────┤
│ Offset │  Size  │                  Field                         │  Type   │
├────────┼────────┼────────────────────────────────────────────────┼─────────┤
│ HEADER (8 bytes)                                                           │
├────────┼────────┼────────────────────────────────────────────────┼─────────┤
│   0    │   2    │ Magic number (0x55AA)                          │ uint16  │
│   2    │   2    │ Sequence number                                │ uint16  │
│   4    │   4    │ Timestamp (µs since boot)                      │ uint32  │
├────────┼────────┼────────────────────────────────────────────────┼─────────┤
│ IMU DATA (28 bytes)                                                        │
├────────┼────────┼────────────────────────────────────────────────┼─────────┤
│   8    │   4    │ Gyro X (rad/s)                                 │ float   │
│  12    │   4    │ Gyro Y (rad/s)                                 │ float   │
│  16    │   4    │ Gyro Z (rad/s)                                 │ float   │
│  20    │   4    │ Accel X (m/s²)                                 │ float   │
│  24    │   4    │ Accel Y (m/s²)                                 │ float   │
│  28    │   4    │ Accel Z (m/s²)                                 │ float   │
│  32    │   4    │ IMU Temperature (°C)                           │ float   │
├────────┼────────┼────────────────────────────────────────────────┼─────────┤
│ MAG DATA (12 bytes)                                                        │
├────────┼────────┼────────────────────────────────────────────────┼─────────┤
│  36    │   4    │ Mag X (Gauss)                                  │ float   │
│  40    │   4    │ Mag Y (Gauss)                                  │ float   │
│  44    │   4    │ Mag Z (Gauss)                                  │ float   │
├────────┼────────┼────────────────────────────────────────────────┼─────────┤
│ BARO DATA (8 bytes)                                                        │
├────────┼────────┼────────────────────────────────────────────────┼─────────┤
│  48    │   4    │ Pressure (Pa)                                  │ float   │
│  52    │   4    │ Baro Altitude (m)                              │ float   │
├────────┼────────┼────────────────────────────────────────────────┼─────────┤
│ GPS DATA (24 bytes)                                                        │
├────────┼────────┼────────────────────────────────────────────────┼─────────┤
│  56    │   4    │ Latitude (deg × 1e7)                           │ int32   │
│  60    │   4    │ Longitude (deg × 1e7)                          │ int32   │
│  64    │   4    │ Altitude MSL (mm)                              │ int32   │
│  68    │   4    │ Ground speed (cm/s)                            │ uint32  │
│  72    │   2    │ Heading (deg × 100)                            │ int16   │
│  74    │   1    │ Fix type (0=none, 2=2D, 3=3D)                  │ uint8   │
│  75    │   1    │ Satellites visible                             │ uint8   │
│  76    │   2    │ HDOP (× 100)                                   │ uint16  │
│  78    │   2    │ VDOP (× 100)                                   │ uint16  │
├────────┼────────┼────────────────────────────────────────────────┼─────────┤
│ ATTITUDE / EKF OUTPUT (32 bytes)                                           │
├────────┼────────┼────────────────────────────────────────────────┼─────────┤
│  80    │   4    │ Quaternion W                                   │ float   │
│  84    │   4    │ Quaternion X                                   │ float   │
│  88    │   4    │ Quaternion Y                                   │ float   │
│  92    │   4    │ Quaternion Z                                   │ float   │
│  96    │   4    │ Roll (rad)                                     │ float   │
│ 100    │   4    │ Pitch (rad)                                    │ float   │
│ 104    │   4    │ Yaw (rad)                                      │ float   │
│ 108    │   4    │ EKF innovation variance                        │ float   │
├────────┼────────┼────────────────────────────────────────────────┼─────────┤
│ SYSTEM STATUS (16 bytes)                                                   │
├────────┼────────┼────────────────────────────────────────────────┼─────────┤
│ 112    │   1    │ Health level (0=GOOD, 1=WARN, 2=CRIT, 3=FAIL) │ uint8   │
│ 113    │   1    │ Healthy IMU bitmask                            │ uint8   │
│ 114    │   1    │ Sensor status flags                            │ uint8   │
│ 115    │   1    │ Reserved                                       │ uint8   │
│ 116    │   4    │ CPU load (0-1000 = 0-100.0%)                   │ uint16  │
│ 118    │   2    │ Battery voltage (mV)                           │ uint16  │
│ 120    │   4    │ Loop count                                     │ uint32  │
├────────┼────────┼────────────────────────────────────────────────┼─────────┤
│ FOOTER (4 bytes)                                                           │
├────────┼────────┼────────────────────────────────────────────────┼─────────┤
│ 124    │   2    │ CRC16 (CCITT)                                  │ uint16  │
│ 126    │   2    │ End marker (0xAA55)                            │ uint16  │
└────────┴────────┴────────────────────────────────────────────────┴─────────┘

Total: 128 bytes (power of 2 for efficient DMA)
```

### 2.2 Packet Sync Strategy

```
┌────────────────────────────────────────────────────────────────────────────┐
│                         PACKET SYNCHRONIZATION                             │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│  State Machine cho Receiver:                                               │
│                                                                            │
│    ┌──────────┐     0x55      ┌──────────┐     0xAA      ┌──────────┐     │
│    │  SYNC_1  │ ───────────▶  │  SYNC_2  │ ───────────▶  │ HEADER   │     │
│    └──────────┘               └──────────┘               └──────────┘     │
│         ▲                          │                          │            │
│         │                          │ !=0xAA                   │ 6 bytes    │
│         │                          ▼                          ▼            │
│         └──────────────────── RESYNC ◀──────────────────  ┌──────────┐    │
│                                   ▲                       │ PAYLOAD  │    │
│                                   │                       └──────────┘    │
│                                   │ CRC fail                  │            │
│                                   │                           │ 116 bytes  │
│                                   │                           ▼            │
│                               ┌──────────┐  CRC OK    ┌──────────────┐    │
│                               │ VALIDATE │ ◀──────────│    FOOTER    │    │
│                               └──────────┘            └──────────────┘    │
│                                   │                                        │
│                                   │ valid                                  │
│                                   ▼                                        │
│                               ┌──────────┐                                 │
│                               │ COMPLETE │ ─▶ Dispatch to data buffer     │
│                               └──────────┘                                 │
│                                                                            │
│  Recovery Strategy:                                                        │
│  - Nếu mất sync: Scan byte-by-byte tìm magic 0x55AA                       │
│  - Validate sequence continuity (detect drops)                             │
│  - Timeout 100ms: Reset state machine                                      │
│                                                                            │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. KIẾN TRÚC PC TOOL

### 3.1 Layer Architecture

```
┌────────────────────────────────────────────────────────────────────────────┐
│                              LAYER DIAGRAM                                 │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│  ┌─────────────────────────────────────────────────────────────────────┐  │
│  │                     PRESENTATION LAYER                               │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────┐ │  │
│  │  │ MainWindow  │  │ 3D Viewer   │  │ Plot Panel  │  │ Data Table │ │  │
│  │  │  (PySide6)  │  │   (VisPy)   │  │ (PyQtGraph) │  │  (QTable)  │ │  │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  └────────────┘ │  │
│  └─────────────────────────────────────────────────────────────────────┘  │
│                                    ▲                                       │
│                                    │ Qt Signals                            │
│  ┌─────────────────────────────────┼───────────────────────────────────┐  │
│  │                     APPLICATION LAYER                                │  │
│  │  ┌─────────────────────────────────────────────────────────────────┐│  │
│  │  │                      DataManager                                 ││  │
│  │  │  - Coordinates all data flow                                     ││  │
│  │  │  - Emits Qt signals for GUI updates                              ││  │
│  │  │  - Manages logging                                               ││  │
│  │  └─────────────────────────────────────────────────────────────────┘│  │
│  └─────────────────────────────────────────────────────────────────────┘  │
│                                    ▲                                       │
│                                    │                                       │
│  ┌─────────────────────────────────┼───────────────────────────────────┐  │
│  │                       DATA LAYER                                     │  │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐ │  │
│  │  │   DataBuffer    │  │   PacketStats   │  │     DataLogger      │ │  │
│  │  │  (Ring Buffer)  │  │  (Drop count)   │  │     (SQLite)        │ │  │
│  │  │  Thread-safe    │  │                 │  │                     │ │  │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────────┘ │  │
│  └─────────────────────────────────────────────────────────────────────┘  │
│                                    ▲                                       │
│                                    │                                       │
│  ┌─────────────────────────────────┼───────────────────────────────────┐  │
│  │                     PROTOCOL LAYER                                   │  │
│  │  ┌─────────────────────────────────────────────────────────────────┐│  │
│  │  │                     PacketDecoder                                ││  │
│  │  │  - State machine for sync                                        ││  │
│  │  │  - CRC validation                                                ││  │
│  │  │  - Struct unpacking                                              ││  │
│  │  └─────────────────────────────────────────────────────────────────┘│  │
│  └─────────────────────────────────────────────────────────────────────┘  │
│                                    ▲                                       │
│                                    │                                       │
│  ┌─────────────────────────────────┼───────────────────────────────────┐  │
│  │                     TRANSPORT LAYER                                  │  │
│  │  ┌─────────────────────────────────────────────────────────────────┐│  │
│  │  │                    SerialReceiver                                ││  │
│  │  │  - Dedicated thread                                              ││  │
│  │  │  - Non-blocking read                                             ││  │
│  │  │  - Raw byte stream to decoder                                    ││  │
│  │  └─────────────────────────────────────────────────────────────────┘│  │
│  └─────────────────────────────────────────────────────────────────────┘  │
│                                                                            │
└────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Thread Model

```
┌────────────────────────────────────────────────────────────────────────────┐
│                           THREAD MODEL                                     │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│  ┌───────────────────────────────────────────────────────────────────────┐│
│  │                         MAIN THREAD (GUI)                             ││
│  │  - Qt Event Loop                                                      ││
│  │  - All widget updates                                                 ││
│  │  - User input handling                                                ││
│  │  - Timer-based refresh (60 Hz for GUI, 30 Hz for plots)              ││
│  └───────────────────────────────────────────────────────────────────────┘│
│                     ▲                                                      │
│                     │ Qt Signals (thread-safe)                            │
│                     │                                                      │
│  ┌──────────────────┴────────────────────────────────────────────────────┐│
│  │                      RECEIVER THREAD                                   ││
│  │  - Dedicated for serial I/O                                           ││
│  │  - Non-blocking read with select()                                    ││
│  │  - Feeds PacketDecoder                                                ││
│  │  - Emits signals on valid packet                                      ││
│  │  - No GUI interaction                                                 ││
│  └───────────────────────────────────────────────────────────────────────┘│
│                                                                            │
│  ┌───────────────────────────────────────────────────────────────────────┐│
│  │                       LOGGER THREAD                                    ││
│  │  - Async write to SQLite                                              ││
│  │  - Queue-based                                                        ││
│  │  - Low priority                                                       ││
│  └───────────────────────────────────────────────────────────────────────┘│
│                                                                            │
│  Communication:                                                            │
│  - Receiver → GUI: Qt signals with TelemetryData object                   │
│  - GUI → Logger: Queue.put() (non-blocking)                               │
│  - No shared mutable state between threads                                 │
│                                                                            │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. GUI LAYOUT

### 4.1 Main Window Layout

```
┌────────────────────────────────────────────────────────────────────────────┐
│  File   View   Tools   Help                                    [_][□][X]  │
├────────────────────────────────────────────────────────────────────────────┤
│  [▶ Connect] [■ Stop] [⏺ Record] [⏹ Stop Rec]    Port: [/dev/ttyUSB0 ▼]  │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│  ┌──────────────────────────┐  ┌─────────────────────────────────────────┐│
│  │     CONTROL PANEL        │  │           3D ATTITUDE VIEW              ││
│  │  ┌────────────────────┐  │  │                                         ││
│  │  │ Connection Status  │  │  │         ╱───────────╲                   ││
│  │  │ ● Connected        │  │  │        ╱             ╲                  ││
│  │  │ Packets: 12,345    │  │  │      ╔═══════════════════╗              ││
│  │  │ Drops: 2           │  │  │      ║                   ║              ││
│  │  │ Latency: 5.2 ms    │  │  │      ║     UAV Model     ║              ││
│  │  └────────────────────┘  │  │      ║                   ║              ││
│  │                          │  │      ╚═══════════════════╝              ││
│  │  ┌────────────────────┐  │  │        ╲             ╱                  ││
│  │  │ System Health      │  │  │         ╲───────────╱                   ││
│  │  │ ● IMU: 4/4 OK      │  │  │                                         ││
│  │  │ ● Baro: OK         │  │  │   Roll: +2.5°  Pitch: -1.2°  Yaw: 45.3°││
│  │  │ ● Mag: OK          │  │  │                                         ││
│  │  │ ● GPS: 3D Fix (12) │  │  │   Q: [0.924, 0.015, -0.010, 0.382]     ││
│  │  │ ● EKF: Converged   │  │  │                                         ││
│  │  └────────────────────┘  │  └─────────────────────────────────────────┘│
│  │                          │                                              │
│  │  ┌────────────────────┐  │                                              │
│  │  │ CPU: 23.5%         │  │                                              │
│  │  │ Battery: 11.8V     │  │                                              │
│  │  │ Uptime: 00:15:32   │  │                                              │
│  │  └────────────────────┘  │                                              │
│  └──────────────────────────┘                                              │
│                                                                            │
├────────────────────────────────────────────────────────────────────────────┤
│                          REALTIME DATA PLOTS                               │
│  ┌────────────────────────────────────────────────────────────────────────┐│
│  │  [IMU ▼]  [Attitude ▼]  [Baro ▼]  [GPS ▼]  [EKF ▼]   Time: [10s ▼]  ││
│  ├────────────────────────────────────────────────────────────────────────┤│
│  │                                                                        ││
│  │  Gyro X ────────────────────────────────────────────────────── 0.02   ││
│  │  Gyro Y ────────────────────────────────────────────────────── 0.01   ││
│  │  Gyro Z ────────────────────────────────────────────────────── -0.03  ││
│  │         ─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬───     ││
│  │              -10s  -8s   -6s   -4s   -2s   now                        ││
│  │                                                                        ││
│  │  Accel X ───────────────────────────────────────────────────── 0.15   ││
│  │  Accel Y ───────────────────────────────────────────────────── -0.08  ││
│  │  Accel Z ───────────────────────────────────────────────────── -9.79  ││
│  │                                                                        ││
│  └────────────────────────────────────────────────────────────────────────┘│
│                                                                            │
├────────────────────────────────────────────────────────────────────────────┤
│                           DATA TABLE                                       │
│  ┌────────────────────────────────────────────────────────────────────────┐│
│  │ Sensor      │    X       │    Y       │    Z       │   Unit   │ Status││
│  ├─────────────┼────────────┼────────────┼────────────┼──────────┼────────┤│
│  │ Gyro        │   0.0234   │   0.0123   │  -0.0312   │  rad/s   │   ●   ││
│  │ Accel       │   0.1523   │  -0.0812   │  -9.7923   │  m/s²    │   ●   ││
│  │ Mag         │   0.2341   │   0.1234   │  -0.4521   │  Gauss   │   ●   ││
│  │ Baro Alt    │       --   │       --   │  125.34    │    m     │   ●   ││
│  │ GPS Pos     │  21.028... │ 105.834... │   45.2     │  deg/m   │   ●   ││
│  └────────────────────────────────────────────────────────────────────────┘│
│                                                                            │
├────────────────────────────────────────────────────────────────────────────┤
│  Ready │ Rx: 1234.5 KB │ Packets: 98,765 │ Drops: 12 (0.01%) │ FPS: 60   │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 5. FILE STRUCTURE

```
tools/
├── uav_gcs/                          # Ground Control Station tool
│   ├── __init__.py
│   ├── main.py                       # Entry point
│   │
│   ├── protocol/                     # Protocol layer
│   │   ├── __init__.py
│   │   ├── packet.py                 # Packet definition & CRC
│   │   ├── decoder.py                # State machine decoder
│   │   └── serial_receiver.py        # Serial I/O thread
│   │
│   ├── data/                         # Data layer
│   │   ├── __init__.py
│   │   ├── data_buffer.py            # Thread-safe ring buffer
│   │   ├── data_manager.py           # Coordinator
│   │   └── logger.py                 # SQLite logger
│   │
│   ├── gui/                          # Presentation layer
│   │   ├── __init__.py
│   │   ├── main_window.py            # Main window
│   │   ├── control_panel.py          # Left panel
│   │   ├── attitude_view.py          # 3D visualization
│   │   ├── plot_panel.py             # PyQtGraph plots
│   │   ├── data_table.py             # Data table
│   │   └── resources/                # Icons, models
│   │       ├── uav_model.obj         # 3D model
│   │       └── icons/
│   │
│   ├── utils/                        # Utilities
│   │   ├── __init__.py
│   │   ├── crc16.py                  # CRC16-CCITT
│   │   └── quaternion.py             # Quaternion math
│   │
│   └── config.py                     # Configuration
│
├── requirements.txt                  # Python dependencies
└── run_gcs.py                        # Launcher script
```

---

## 6. LOGGING FORMAT

### 6.1 SQLite Schema

```sql
-- Metadata table
CREATE TABLE sessions (
    id INTEGER PRIMARY KEY,
    start_time TEXT,
    end_time TEXT,
    description TEXT,
    firmware_version TEXT
);

-- Telemetry data
CREATE TABLE telemetry (
    id INTEGER PRIMARY KEY,
    session_id INTEGER,
    timestamp_us INTEGER,     -- MCU timestamp
    pc_time_ms INTEGER,       -- PC receive time
    sequence INTEGER,
    
    -- IMU
    gyro_x REAL, gyro_y REAL, gyro_z REAL,
    accel_x REAL, accel_y REAL, accel_z REAL,
    imu_temp REAL,
    
    -- Mag
    mag_x REAL, mag_y REAL, mag_z REAL,
    
    -- Baro
    pressure REAL, baro_alt REAL,
    
    -- GPS
    latitude INTEGER, longitude INTEGER, altitude_msl INTEGER,
    ground_speed INTEGER, heading INTEGER,
    fix_type INTEGER, satellites INTEGER,
    
    -- Attitude
    qw REAL, qx REAL, qy REAL, qz REAL,
    roll REAL, pitch REAL, yaw REAL,
    
    -- Status
    health_level INTEGER,
    cpu_load INTEGER,
    
    FOREIGN KEY (session_id) REFERENCES sessions(id)
);

-- Index for fast time-based queries
CREATE INDEX idx_telemetry_time ON telemetry(session_id, timestamp_us);
```

---

## 7. DEPENDENCIES

### 7.1 Python Requirements

```
# requirements.txt

# GUI Framework
PySide6>=6.5.0

# Plotting
pyqtgraph>=0.13.0
numpy>=1.24.0

# 3D Visualization
vispy>=0.14.0
PyOpenGL>=3.1.0

# Serial Communication
pyserial>=3.5

# Database
# (SQLite built-in)

# Utilities
# (struct, collections built-in)
```

---

## 8. PERFORMANCE TARGETS

| Metric                    | Target         | Measurement Method              |
|--------------------------|----------------|--------------------------------|
| GUI Frame Rate           | ≥ 60 FPS       | Qt frame timing                |
| Plot Update Rate         | ≥ 30 FPS       | PyQtGraph timer                |
| 3D Render Rate           | ≥ 30 FPS       | VisPy frame timing             |
| Packet Decode Latency    | < 1 ms         | Timestamp delta                |
| End-to-end Latency       | < 15 ms        | MCU timestamp vs PC receive    |
| Memory Usage             | < 500 MB       | Process monitor                |
| CPU Usage (PC)           | < 30%          | Process monitor                |
| Packet Drop Rate         | < 0.1%         | Sequence number analysis       |

---

## 9. ERROR HANDLING

| Error Type               | Detection                      | Recovery Action               |
|--------------------------|--------------------------------|-------------------------------|
| Serial disconnection     | IOError on read                | Auto-reconnect with backoff   |
| CRC failure              | CRC16 mismatch                 | Discard packet, resync        |
| Sequence gap             | seq_new != seq_old + 1         | Log drop count, continue      |
| GUI freeze               | Timer callback missed          | Reduce update rate            |
| Buffer overflow          | Ring buffer full               | Overwrite oldest              |
| Out of memory            | MemoryError                    | Reduce history length         |

---

**Tiếp theo: Implementation chi tiết từng module.**
