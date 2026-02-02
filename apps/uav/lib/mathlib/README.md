# lib/mathlib - Thư Viện Toán Học

## 📋 Tổng Quan

Module `mathlib` cung cấp các **classes và hàm toán học** cơ bản cho điều khiển bay:
- **Vector3:** Vector 3D (vận tốc, vị trí, tốc độ góc)
- **Matrix3:** Ma trận 3×3 (rotation, transformation)
- **Quaternion:** Biểu diễn tư thế (attitude)
- **Filters:** Bộ lọc tần số thấp

---

## 📁 Files

```
mathlib/
├── Makefile
├── mathlib.hpp      # Master include + constants + utility functions
├── vector3.hpp      # 3D vector class
├── matrix3.hpp      # 3×3 matrix class
├── quaternion.hpp   # Quaternion class
└── filters.hpp      # Filter utilities
```

---

## 🧭 mathlib.hpp - Constants & Utilities

### Hằng Số

```cpp
#include <uav/lib/mathlib/mathlib.hpp>
using namespace mathlib;

// Chuyển đổi góc
DEG_TO_RAD   // = 0.01745... (π/180)
RAD_TO_DEG   // = 57.2957... (180/π)

// Gia tốc trọng trường chuẩn
GRAVITY_MSS  // = 9.80665 m/s²
```

### Utility Functions

```cpp
// Giới hạn giá trị trong khoảng
float val = constrain(x, -10.0f, 10.0f);

// Wrap góc về [-π, π]
float angle = wrap_pi(raw_angle);

// Wrap góc về [0, 2π]
float angle = wrap_2pi(raw_angle);

// Nội suy tuyến tính
float result = lerp(a, b, t);  // a + t*(b-a)

// Căn bậc 2 an toàn (trả về 0 nếu input < 0)
float r = safe_sqrt(x);

// Dấu của số
float s = sign(x);  // -1, 0, hoặc +1
```

---

## 📐 Vector3 - Vector 3D

### Mục Đích

Biểu diễn đại lượng 3D trong không gian:
- **Vận tốc:** $(v_x, v_y, v_z)$
- **Gia tốc:** $(a_x, a_y, a_z)$
- **Tốc độ góc:** $(\omega_x, \omega_y, \omega_z)$
- **Vị trí:** $(x, y, z)$

### API

```cpp
#include <uav/lib/mathlib/vector3.hpp>
using namespace mathlib;

// Khởi tạo
Vector3 v;                    // (0, 0, 0)
Vector3 v(1.0f, 2.0f, 3.0f); // (1, 2, 3)
Vector3 v(array);             // Từ float[3]

// Truy cập components
float x = v.x;
float y = v.y;
float z = v.z;
float val = v[i];  // i = 0, 1, 2

// Độ lớn (norm)
float len = v.norm();           // √(x²+y²+z²)
float len_sq = v.norm_squared(); // x²+y²+z² (tránh sqrt)

// Chuẩn hóa
v.normalize();                // In-place
Vector3 unit = v.normalized(); // Trả về bản sao

// Tích vô hướng (dot product)
float dot = v1.dot(v2);  // v1·v2 = x1*x2 + y1*y2 + z1*z2

// Tích có hướng (cross product)
Vector3 c = v1.cross(v2);  // v1 × v2

// Operators
Vector3 sum = v1 + v2;
Vector3 diff = v1 - v2;
Vector3 scaled = v * 2.0f;
Vector3 neg = -v;
v1 += v2;
v *= 2.0f;
```

### Ứng Dụng

```cpp
// Tính gia tốc ly tâm: a = ω × (ω × r)
Vector3 omega(gyro[0], gyro[1], gyro[2]);
Vector3 r(0, 0, lever_arm);
Vector3 centripetal = omega.cross(omega.cross(r));

// Chuẩn hóa vector gia tốc để lấy hướng
Vector3 accel_raw(ax, ay, az);
Vector3 accel_dir = accel_raw.normalized();

// Góc giữa 2 vectors
float cos_angle = v1.normalized().dot(v2.normalized());
float angle = acosf(cos_angle);
```

---

## 🔲 Matrix3 - Ma Trận 3×3

### Mục Đích

- **Rotation Matrix (DCM):** Direction Cosine Matrix
- **Transformation:** Xoay vectors giữa các frame
- **Skew-symmetric:** Dùng trong tính toán cross product

### Storage

```
Row-major: m[row][col]

┌─────────────────────────────────┐
│ m[0][0]  m[0][1]  m[0][2]       │  Row 0
│ m[1][0]  m[1][1]  m[1][2]       │  Row 1
│ m[2][0]  m[2][1]  m[2][2]       │  Row 2
└─────────────────────────────────┘
   Col 0    Col 1    Col 2
```

### API

```cpp
#include <uav/lib/mathlib/matrix3.hpp>
using namespace mathlib;

// Khởi tạo (mặc định: Identity)
Matrix3 M;  // I₃

// Từ Euler angles (ZYX convention)
Matrix3 R = Matrix3::from_euler(roll, pitch, yaw);

// Skew-symmetric matrix từ vector
// [v]ₓ sao cho [v]ₓ * u = v × u
float v[3] = {vx, vy, vz};
Matrix3 S = Matrix3::skew(v);

// Transpose (đổi hàng/cột)
Matrix3 T = M.transpose();

// Determinant
float d = M.det();

// Inverse (với rotation matrix: inverse = transpose)
Matrix3 inv = M.inverse();

// Nhân matrix
Matrix3 C = A * B;

// Transform vector
float v_in[3] = {...};
float v_out[3];
M.transform(v_in, v_out);  // v_out = M * v_in
```

### Rotation Matrix (DCM)

```cpp
// Body → Inertial frame transformation
Matrix3 R_body_to_inertial = Matrix3::from_euler(roll, pitch, yaw);

// Transform acceleration từ body frame sang inertial
float accel_body[3] = {ax, ay, az};
float accel_inertial[3];
R_body_to_inertial.transform(accel_body, accel_inertial);

// Inertial → Body: dùng transpose
Matrix3 R_inertial_to_body = R_body_to_inertial.transpose();
```

---

## 🔄 Quaternion - Biểu Diễn Tư Thế

### Tại Sao Dùng Quaternion?

| Phương Pháp | Gimbal Lock | Memory | Computation |
|-------------|-------------|--------|-------------|
| Euler Angles | ⚠️ Có (±90° pitch) | 12 bytes | Nhiều sin/cos |
| Rotation Matrix | ✅ Không | 36 bytes | Nhân ma trận |
| **Quaternion** | ✅ Không | **16 bytes** | **Ít nhất** |

### Convention

```
Hamilton convention: q = w + xi + yj + zk

q = [w, x, y, z]
    ↑     └─────── Vector part (3D)
    └──────────── Scalar part

Unit quaternion: |q| = √(w² + x² + y² + z²) = 1
```

### API

```cpp
#include <uav/lib/mathlib/quaternion.hpp>
using namespace mathlib;

// Khởi tạo (mặc định: Identity)
Quaternion q;  // [1, 0, 0, 0] = no rotation

// Từ Euler angles (ZYX convention)
Quaternion q = Quaternion::from_euler(roll, pitch, yaw);

// Từ axis-angle
float axis[3] = {0, 0, 1};  // Z-axis
float angle = M_PI / 4;      // 45 degrees
Quaternion q = Quaternion::from_axis_angle(axis, angle);

// Chuyển sang Euler
float roll, pitch, yaw;
q.to_euler(&roll, &pitch, &yaw);

// Chuyển sang DCM
float R[9];
q.to_dcm(R);  // Row-major 3×3 matrix

// Norm và normalize
float n = q.norm();
q.normalize();

// Conjugate (inverse for unit quaternion)
Quaternion q_inv = q.conjugate();

// Nhân quaternion: q3 = q1 * q2
// Biểu diễn q2 rotation rồi q1 rotation
Quaternion q3 = q1 * q2;

// Rotate vector
float v_in[3] = {...};
float v_out[3];
q.rotate_vector(v_in, v_out);
```

### Gyro Integration

Cập nhật quaternion từ tốc độ góc (gyro):

```cpp
// Quaternion rate: dq/dt = 0.5 * q * [0, ωx, ωy, ωz]
void integrate_gyro(Quaternion& q, const float gyro[3], float dt)
{
    // Quaternion from angular velocity
    float half_dt = 0.5f * dt;
    Quaternion dq(
        1.0f,
        gyro[0] * half_dt,
        gyro[1] * half_dt,
        gyro[2] * half_dt
    );
    
    // Update: q_new = q * dq
    q = q * dq;
    q.normalize();  // Giữ unit quaternion
}
```

### Slerp (Spherical Linear Interpolation)

```cpp
// Nội suy mượt giữa 2 orientations
Quaternion q_interp = Quaternion::slerp(q1, q2, t);
// t = 0 → q1
// t = 1 → q2
// 0 < t < 1 → nội suy

// Ứng dụng: smooth attitude transitions
```

---

## 📊 Ví Dụ Tổng Hợp

### EKF Attitude Update

```cpp
#include <uav/lib/mathlib/mathlib.hpp>

using namespace mathlib;

class AttitudeEstimator {
    Quaternion q;           // Current attitude
    Vector3 accel_bias;     // Accelerometer bias
    Vector3 gyro_bias;      // Gyroscope bias
    
public:
    void predict(const float gyro[3], float dt) {
        // Gyro integration
        Vector3 omega(gyro[0] - gyro_bias.x,
                      gyro[1] - gyro_bias.y,
                      gyro[2] - gyro_bias.z);
        
        // Update quaternion
        float half_dt = 0.5f * dt;
        Quaternion dq(1.0f,
                      omega.x * half_dt,
                      omega.y * half_dt,
                      omega.z * half_dt);
        q = q * dq;
        q.normalize();
    }
    
    void correct(const float accel[3]) {
        // Get gravity direction in body frame
        Vector3 accel_meas(accel[0], accel[1], accel[2]);
        accel_meas.normalize();
        
        // Expected gravity (down) in body frame
        float g_body[3];
        float g_inertial[3] = {0, 0, -1};  // -Z = down
        q.conjugate().rotate_vector(g_inertial, g_body);
        Vector3 accel_pred(g_body);
        
        // Error: cross product
        Vector3 error = accel_meas.cross(accel_pred);
        
        // Correct attitude (simplified)
        Quaternion dq(1.0f,
                      error.x * K_P,
                      error.y * K_P,
                      error.z * K_P);
        q = q * dq;
        q.normalize();
    }
};
```

---

## 📈 Performance

| Operation | Cycles (Cortex-M7) |
|-----------|-------------------|
| Vector3 add | 3 |
| Vector3 dot | 6 |
| Vector3 cross | 12 |
| Vector3 normalize | 25-30 |
| Quaternion multiply | 28 |
| Quaternion normalize | 30-35 |
| Matrix3 multiply | 54 |
| Matrix3 transform | 18 |

---

## 🔗 Liên Kết

- **Usage:** `/apps/uav/estimator_app/`
- **DSP Filters:** `/apps/uav/lib/dsp/`
