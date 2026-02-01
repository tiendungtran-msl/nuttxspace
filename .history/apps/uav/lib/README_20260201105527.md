# Shared Libraries - Thư viện dùng chung

## Tổng quan

Thư mục này chứa các thư viện toán học và tiện ích dùng chung
giữa các app.

## Thư viện

### 1. mathlib - Thư viện toán học

```
lib/mathlib/
├── matrix.hpp          # Matrix operations
├── vector.hpp          # Vector operations
├── quaternion.hpp      # Quaternion class
├── rotation.hpp        # Rotation utilities
└── filters.hpp         # Digital filters
```

**Mục đích:**
- Quaternion multiplication, inverse, to/from Euler
- Matrix 3x3, 4x4 operations
- Vector operations (cross, dot, normalize)
- Low-pass, high-pass filters

### 2. geo - Thư viện địa lý

```
lib/geo/
├── geo.hpp             # Coordinate conversions
└── projection.hpp      # Map projections
```

**Mục đích:**
- LLA to ECEF conversion
- ECEF to NED conversion
- Great circle distance

### 3. ecl - Estimation and Control Library

```
lib/ecl/
├── EKF/                # EKF2 implementation
│   ├── ekf.hpp
│   ├── ekf.cpp
│   └── ...
└── attitude/           # Attitude estimator
    └── ...
```

**Mục đích:**
- Port từ PX4 ECL
- EKF2 algorithm
- Attitude estimator

## Build

Các lib được build thành static library:
- `libuav_mathlib.a`
- `libuav_geo.a`
- `libuav_ecl.a`

## Usage

```cpp
#include <uav/lib/mathlib/quaternion.hpp>

using namespace mathlib;

Quaternion q = Quaternion::from_euler(0.1f, 0.2f, 0.3f);
float roll, pitch, yaw;
q.to_euler(&roll, &pitch, &yaw);
```
