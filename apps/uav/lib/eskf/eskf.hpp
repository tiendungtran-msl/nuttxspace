/****************************************************************************
 * apps/uav/lib/eskf/eskf.hpp
 *
 * Error-State Kalman Filter (ESKF) cho ước lượng tư thế
 *
 * THUẬT TOÁN:
 * Dựa trên:
 * - J. Solà, "Quaternion kinematics for the error-state Kalman filter"
 *   (Sections 4.4, 7.1, 7.2, 7.3)
 * - PX4 EKF2 architecture (error-state formulation)
 * - ESKF_IMU_GPS_Fusion reference (covariance propagation)
 *
 * VECTOR TRẠNG THÁI (9 phần tử, không GPS):
 *
 * Trạng thái danh nghĩa (nominal state):
 *   - q: Quaternion body→NED (4 thành phần)
 *   - bg: Gyro bias (3)
 *   - ba: Accel bias (3)
 *
 * Trạng thái sai số (error state, 9×1):
 *   δx = [δθ(3), δbg(3), δba(3)]^T
 *   - δθ: Sai số tư thế (rotation vector) [rad]
 *   - δbg: Sai số gyro bias [rad/s]
 *   - δba: Sai số accel bias [m/s²]
 *
 * Ma trận hiệp phương sai P: 9×9
 *
 * PREDICTION (mỗi mẫu IMU, ~1kHz):
 *   Nominal: q ← q ⊗ Δq(ω_c × dt)
 *   Error:   P ← Fx × P × Fx^T + Q
 *
 *   Fx (9×9) = | R{Δθ}^T   -I·dt    0   |
 *              |    0        I       0   |
 *              |    0        0       I   |
 *
 * UPDATE (khi quasi-static, accelerometer đo trọng lực):
 *   Measurement: a_meas ≈ -R^T × g_ned + ba
 *   Jacobian:    H = [-[g_body]×,  0₃ₓ₃,  I₃ₓ₃]
 *   Joseph form: P = (I-KH)P(I-KH)^T + KRK^T
 *
 * CÁC TRẠNG THÁI CÓ THỂ QUAN SÁT (không GPS/Mag):
 *   ✓ Roll, Pitch (từ trọng lực)
 *   ✓ Gyro bias (từ hiệu chỉnh tư thế)
 *   ✓ Accel bias (từ chuẩn trọng lực)
 *   ✗ Yaw (trôi tự do, không có mag/GPS)
 *
 ****************************************************************************/

#ifndef UAV_LIB_ESKF_ESKF_HPP
#define UAV_LIB_ESKF_ESKF_HPP

#include "eskf_matrix.hpp"
#include <uav/lib/mathlib/quaternion.hpp>

namespace eskf_math {
    static constexpr float DEG_TO_RAD = 0.017453292519943295f;
    template<typename T>
    inline T constrain(T val, T lo, T hi) {
        return (val < lo) ? lo : ((val > hi) ? hi : val);
    }
}

namespace eskf {

/****************************************************************************
 * Configuration
 ****************************************************************************/

struct EskfConfig {
    /* Noise parameters (Power Spectral Density) */
    float gyro_noise;          ///< Gyro white noise [rad/s/√Hz]
    float accel_noise;         ///< Accel white noise [m/s²/√Hz]
    float gyro_bias_noise;     ///< Gyro bias random walk [rad/s²/√Hz]
    float accel_bias_noise;    ///< Accel bias random walk [m/s³/√Hz]

    /* Physical parameters */
    float gravity;             ///< Gravity magnitude [m/s²]

    /* Gate parameters */
    float accel_gate;          ///< Accel magnitude gate (|‖a‖/g - 1| threshold)

    /* Initial uncertainties */
    float init_att_std;        ///< Initial attitude std [rad]
    float init_gyro_bias_std;  ///< Initial gyro bias std [rad/s]
    float init_accel_bias_std; ///< Initial accel bias std [m/s²]
};

/**
 * Cấu hình mặc định cho ICM-42688P
 * Conservative values cho môi trường UAV (có rung động)
 */
inline EskfConfig eskf_default_config() {
    EskfConfig cfg;
    cfg.gyro_noise          = 0.015f;     // rad/s/√Hz
    cfg.accel_noise         = 0.35f;      // m/s²/√Hz
    cfg.gyro_bias_noise     = 0.001f;     // rad/s²/√Hz
    cfg.accel_bias_noise    = 0.01f;      // m/s³/√Hz
    cfg.gravity             = 9.80665f;   // m/s² (standard)
    cfg.accel_gate          = 0.3f;       // ±30% g deviation
    cfg.init_att_std        = 10.0f * eskf_math::DEG_TO_RAD;
    cfg.init_gyro_bias_std  = 0.05f;      // rad/s
    cfg.init_accel_bias_std = 0.5f;       // m/s²
    return cfg;
}

/****************************************************************************
 * State & Output
 ****************************************************************************/

struct EskfState {
    mathlib::Quaternion q;     ///< Attitude quaternion body→NED
    float gyro_bias[3];        ///< Gyro bias [rad/s]
    float accel_bias[3];       ///< Accel bias [m/s²]
    Mat9  P;                   ///< Error-state covariance 9×9
    uint64_t timestamp_us;
    bool  initialized;
};

struct EskfOutput {
    float q[4];                ///< Quaternion [w,x,y,z]
    float roll;                ///< [rad]
    float pitch;               ///< [rad]
    float yaw;                 ///< [rad]
    float gyro_bias[3];        ///< [rad/s]
    float accel_bias[3];       ///< [m/s²]
    float att_cov[3];          ///< Attitude variance diagonal [rad²]
    float gyro_bias_cov[3];    ///< Gyro bias variance
    float accel_bias_cov[3];   ///< Accel bias variance
    bool  valid;
};

/****************************************************************************
 * ESKF Filter Class
 ****************************************************************************/

class Eskf {
public:
    /**
     * Khởi tạo filter với cấu hình
     */
    inline void init(const EskfConfig& config) {
        config_ = config;
        reset();
    }

    /**
     * Reset filter về trạng thái chưa khởi tạo
     */
    inline void reset() {
        state_.q = mathlib::Quaternion();  // Identity
        state_.gyro_bias[0]  = 0.0f;
        state_.gyro_bias[1]  = 0.0f;
        state_.gyro_bias[2]  = 0.0f;
        state_.accel_bias[0] = 0.0f;
        state_.accel_bias[1] = 0.0f;
        state_.accel_bias[2] = 0.0f;
        state_.P.zero();
        state_.timestamp_us  = 0;
        state_.initialized   = false;

        init_count_ = 0;
        for (int i = 0; i < 3; i++) {
            init_accel_sum_[i] = 0.0f;
            init_gyro_sum_[i]  = 0.0f;
        }
    }

    /**
     * Xử lý một mẫu IMU: predict + update
     *
     * @param accel  Gia tốc [m/s²] body frame
     * @param gyro   Vận tốc góc [rad/s] body frame
     * @param timestamp_us  Thời gian [µs since boot]
     * @return true nếu filter có kết quả hợp lệ
     */
    inline bool process_imu(const float accel[3], const float gyro[3],
                            uint64_t timestamp_us) {
        if (!state_.initialized) {
            return try_initialize(accel, gyro, timestamp_us);
        }

        /* Tính dt */
        float dt = (float)(timestamp_us - state_.timestamp_us) * 1e-6f;
        if (dt <= 0.0f || dt > 0.5f) {
            state_.timestamp_us = timestamp_us;
            return true;
        }
        state_.timestamp_us = timestamp_us;

        /* ESKF Prediction (gyro propagation) */
        predict(gyro, dt);

        /* ESKF Update (gravity correction) */
        update_gravity(accel);

        return true;
    }

    /**
     * Lấy kết quả ước lượng hiện tại
     */
    inline void get_output(EskfOutput& out) const {
        state_.q.to_array(out.q);
        state_.q.to_euler(&out.roll, &out.pitch, &out.yaw);

        for (int i = 0; i < 3; i++) {
            out.gyro_bias[i]      = state_.gyro_bias[i];
            out.accel_bias[i]     = state_.accel_bias[i];
            out.att_cov[i]        = state_.P.d[i][i];
            out.gyro_bias_cov[i]  = state_.P.d[3 + i][3 + i];
            out.accel_bias_cov[i] = state_.P.d[6 + i][6 + i];
        }

        out.valid = state_.initialized;
    }

    inline bool     is_initialized() const { return state_.initialized; }
    inline uint64_t timestamp()      const { return state_.timestamp_us; }

private:
    static constexpr int INIT_SAMPLES = 50;

    EskfConfig config_;
    EskfState  state_;

    /* Initialization accumulator */
    float init_accel_sum_[3];
    float init_gyro_sum_[3];
    int   init_count_;

    /*=======================================================================
     * try_initialize - Khởi tạo từ dữ liệu IMU tĩnh
     *
     * Thu thập INIT_SAMPLES mẫu, lấy trung bình:
     * - Accel trung bình → hướng trọng lực → roll, pitch ban đầu
     * - Gyro trung bình → gyro bias ban đầu
     * - Yaw = 0 (không quan sát được)
     *
     * Ref: Solà Section 5.1 "Initialization"
     *======================================================================*/
    inline bool try_initialize(const float accel[3], const float gyro[3],
                               uint64_t timestamp_us) {
        init_accel_sum_[0] += accel[0];
        init_accel_sum_[1] += accel[1];
        init_accel_sum_[2] += accel[2];
        init_gyro_sum_[0]  += gyro[0];
        init_gyro_sum_[1]  += gyro[1];
        init_gyro_sum_[2]  += gyro[2];
        init_count_++;

        if (init_count_ < INIT_SAMPLES) {
            return false;
        }

        float inv_n = 1.0f / (float)init_count_;
        float mean_accel[3] = {
            init_accel_sum_[0] * inv_n,
            init_accel_sum_[1] * inv_n,
            init_accel_sum_[2] * inv_n
        };

        /* Kiểm tra tĩnh: ‖a‖ ≈ g */
        float a_norm = sqrtf(mean_accel[0] * mean_accel[0]
                           + mean_accel[1] * mean_accel[1]
                           + mean_accel[2] * mean_accel[2]);

        if (a_norm < 1.0f) {
            /* Accel data invalid, reset */
            init_count_ = 0;
            for (int i = 0; i < 3; i++) {
                init_accel_sum_[i] = 0.0f;
                init_gyro_sum_[i]  = 0.0f;
            }
            return false;
        }

        if (fabsf(a_norm - config_.gravity) > config_.accel_gate * config_.gravity) {
            /* Không tĩnh, reset */
            init_count_ = 0;
            for (int i = 0; i < 3; i++) {
                init_accel_sum_[i] = 0.0f;
                init_gyro_sum_[i]  = 0.0f;
            }
            return false;
        }

        /*-----------------------------------------------------------------
         * Tính tư thế ban đầu từ hướng trọng lực
         *
         * Khi tĩnh: a_meas = -R^T × g_ned + bias
         * Hướng trọng lực trong body frame: n = -a_meas / ‖a‖
         * n biểu diễn "hướng xuống" (NED down) trong body frame
         *
         * Công thức Euler từ hướng trọng lực:
         *   pitch = asin(-nx)
         *   roll  = atan2(ny, nz)
         *   yaw   = 0 (không quan sát được)
         *----------------------------------------------------------------*/
        float inv_a = 1.0f / a_norm;
        float n[3] = {
            -mean_accel[0] * inv_a,
            -mean_accel[1] * inv_a,
            -mean_accel[2] * inv_a
        };

        /* Clamp nx để tránh asin domain error */
        float nx_clamped = eskf_math::constrain(n[0], -1.0f, 1.0f);

        float pitch = asinf(-nx_clamped);
        float roll  = atan2f(n[1], n[2]);
        float yaw   = 0.0f;

        state_.q = mathlib::Quaternion::from_euler(roll, pitch, yaw);
        state_.q.normalize();

        /* Gyro bias từ trung bình (giả sử tĩnh → gyro đo = bias) */
        state_.gyro_bias[0] = init_gyro_sum_[0] * inv_n;
        state_.gyro_bias[1] = init_gyro_sum_[1] * inv_n;
        state_.gyro_bias[2] = init_gyro_sum_[2] * inv_n;

        /* Accel bias bắt đầu = 0 (khó tách khỏi trọng lực ban đầu) */
        state_.accel_bias[0] = 0.0f;
        state_.accel_bias[1] = 0.0f;
        state_.accel_bias[2] = 0.0f;

        /*-----------------------------------------------------------------
         * Khởi tạo ma trận hiệp phương sai P (9×9)
         *
         * Block diagonal:
         * P(0:3,0:3) = σ²_att     (roll, pitch nhỏ, yaw lớn)
         * P(3:6,3:6) = σ²_gbias
         * P(6:9,6:9) = σ²_abias
         *----------------------------------------------------------------*/
        state_.P.zero();

        float att_var   = config_.init_att_std * config_.init_att_std;
        float gbias_var = config_.init_gyro_bias_std * config_.init_gyro_bias_std;
        float abias_var = config_.init_accel_bias_std * config_.init_accel_bias_std;

        state_.P.d[0][0] = att_var;             // Roll variance
        state_.P.d[1][1] = att_var;             // Pitch variance
        state_.P.d[2][2] = att_var * 100.0f;   // Yaw variance (lớn hơn nhiều)

        state_.P.d[3][3] = gbias_var;
        state_.P.d[4][4] = gbias_var;
        state_.P.d[5][5] = gbias_var;

        state_.P.d[6][6] = abias_var;
        state_.P.d[7][7] = abias_var;
        state_.P.d[8][8] = abias_var;

        state_.timestamp_us = timestamp_us;
        state_.initialized  = true;

        return true;
    }

    /*=======================================================================
     * predict - Bước dự đoán ESKF
     *
     * 1. Cập nhật trạng thái danh nghĩa:
     *    q ← q ⊗ Δq(ω_c × dt)
     *    bg, ba giữ nguyên (mô hình random walk)
     *
     * 2. Truyền ma trận hiệp phương sai:
     *    P ← Fx × P × Fx^T + Q
     *
     * Ma trận chuyển tiếp Fx (9×9):
     * | R{Δθ}^T    -I·dt    0   |   Ref: Solà Eq. (269)
     * |    0          I      0   |
     * |    0          0      I   |
     *
     * R{Δθ}^T tính bằng công thức Rodrigues:
     *   R^T = I - sin(θ)/θ × [Δθ]× + (1-cos(θ))/θ² × [Δθ]×²
     *
     * Ma trận nhiễu Q (9×9):
     * | σ²_ω·dt²·I    0            0           |
     * |    0          σ²_bω·dt·I    0           |
     * |    0           0          σ²_ba·dt·I    |
     *
     *======================================================================*/
    inline void predict(const float gyro[3], float dt) {
        /* Gyro đã bù bias */
        float omega[3] = {
            gyro[0] - state_.gyro_bias[0],
            gyro[1] - state_.gyro_bias[1],
            gyro[2] - state_.gyro_bias[2]
        };

        /* Rotation vector cho bước này */
        float da[3] = {
            omega[0] * dt,
            omega[1] * dt,
            omega[2] * dt
        };

        float angle = sqrtf(da[0] * da[0] + da[1] * da[1] + da[2] * da[2]);

        /*==============================================================
         * 1. Nominal state: quaternion integration
         *    q ← q ⊗ Δq
         *    Δq = [cos(θ/2), sin(θ/2)·e]  với e = da/θ
         *
         * Ref: Solà Eq. (101)
         *=============================================================*/
        mathlib::Quaternion dq;
        if (angle > 1e-8f) {
            float half_angle = angle * 0.5f;
            float s = sinf(half_angle) / angle;
            dq = mathlib::Quaternion(cosf(half_angle),
                                     da[0] * s, da[1] * s, da[2] * s);
        } else {
            /* Xấp xỉ góc nhỏ: Δq ≈ [1, da/2] */
            dq = mathlib::Quaternion(1.0f,
                                     da[0] * 0.5f, da[1] * 0.5f, da[2] * 0.5f);
        }

        state_.q = state_.q * dq;
        state_.q.normalize();

        /*==============================================================
         * 2. Error-state transition matrix Fx (9×9)
         *
         * R{Δθ}^T bằng Rodrigues:
         *   R^T = I - sin(θ)/θ × [da]× + (1-cos(θ))/θ² × [da]×²
         *
         * Ref: Solà Eq. (176), (181)
         *=============================================================*/
        Mat9 Fx;
        Fx.identity();

        if (angle > 1e-8f) {
            float s_a   = sinf(angle) / angle;
            float omc_a2 = (1.0f - cosf(angle)) / (angle * angle);

            Mat3 sk  = skew(da);
            Mat3 sk2 = mat_mul(sk, sk);

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    Fx.d[i][j] = (i == j ? 1.0f : 0.0f)
                                 - s_a * sk.d[i][j]
                                 + omc_a2 * sk2.d[i][j];
                }
            }
        }
        /* else: Fx(0:3,0:3) ≈ I (đã set bởi identity) */

        /* Fx(0:3, 3:6) = -I × dt  (tư thế ← gyro bias) */
        Fx.d[0][3] = -dt;
        Fx.d[1][4] = -dt;
        Fx.d[2][5] = -dt;

        /* Fx(3:6,3:6) = I, Fx(6:9,6:9) = I → đã có từ identity() */

        /*==============================================================
         * 3. Process noise Q (9×9)
         *
         * Ref: Solà Eq. (270-271)
         *=============================================================*/
        float dt2 = dt * dt;

        Mat9 Q;
        Q.zero();

        float gyro_var  = config_.gyro_noise * config_.gyro_noise * dt2;
        float gbias_var = config_.gyro_bias_noise * config_.gyro_bias_noise * dt;
        float abias_var = config_.accel_bias_noise * config_.accel_bias_noise * dt;

        Q.d[0][0] = gyro_var;  Q.d[1][1] = gyro_var;  Q.d[2][2] = gyro_var;
        Q.d[3][3] = gbias_var; Q.d[4][4] = gbias_var; Q.d[5][5] = gbias_var;
        Q.d[6][6] = abias_var; Q.d[7][7] = abias_var; Q.d[8][8] = abias_var;

        /*==============================================================
         * 4. Covariance propagation: P = Fx × P × Fx^T + Q
         *
         * Ref: Solà Eq. (268)
         *=============================================================*/
        Mat9 FxP = mat_mul(Fx, state_.P);
        Mat9 FxT = mat_transpose(Fx);
        state_.P = mat_add(mat_mul(FxP, FxT), Q);

        /* Đảm bảo đối xứng (chống tích lũy lỗi số) */
        mat_symmetrize(state_.P);
    }

    /*=======================================================================
     * update_gravity - Bước cập nhật ESKF bằng quan sát trọng lực
     *
     * Mô hình đo lường (khi quasi-static):
     *   a_meas = -R(q)^T × g_ned + ba + noise
     *   g_ned = [0, 0, g]^T  (trọng lực NED, pointing down)
     *
     * Đạo hàm Jacobian H (3×9):
     *   H = [-[g_body]×,  0₃ₓ₃,  I₃ₓ₃]
     *   trong đó g_body = R^T × g_ned
     *
     * Update EKF tiêu chuẩn:
     *   S = H×P×H^T + R_meas
     *   K = P×H^T × S^(-1)
     *   δx = K × (a_meas - a_pred)
     *   P = (I-KH)P(I-KH)^T + K×R×K^T   (Joseph form)
     *
     * Injection:
     *   q ← q ⊗ Δq(δθ)
     *   bg ← bg + δbg
     *   ba ← ba + δba
     *
     * Ref: Solà Section 7.2, Eq. (281)-(289)
     *======================================================================*/
    inline void update_gravity(const float accel[3]) {
        /*--------------------------------------------------------------
         * Gate: Kiểm tra quasi-static
         * Chỉ update khi ‖a‖ ≈ g (thiết bị không gia tốc mạnh)
         *-------------------------------------------------------------*/
        float a_norm_sq = accel[0] * accel[0]
                        + accel[1] * accel[1]
                        + accel[2] * accel[2];
        float a_norm = sqrtf(a_norm_sq);

        if (a_norm < 0.1f) {
            return; /* Sensor data không hợp lệ */
        }

        float a_error = fabsf(a_norm - config_.gravity) / config_.gravity;

        if (a_error > config_.accel_gate) {
            return; /* Gia tốc động lực quá lớn, bỏ qua gravity correction */
        }

        /*--------------------------------------------------------------
         * 1. Trọng lực dự đoán trong body frame
         *    g_body = R^T × g_ned = R^T × [0, 0, g]
         *
         * Với R row-major từ to_dcm():
         *    g_body = [R[2][0]×g, R[2][1]×g, R[2][2]×g]
         *           = [R[6]×g,    R[7]×g,    R[8]×g]
         *-------------------------------------------------------------*/
        float R[9];
        state_.q.to_dcm(R);

        float g = config_.gravity;
        float g_body[3] = {R[6] * g, R[7] * g, R[8] * g};

        /*--------------------------------------------------------------
         * 2. Gia tốc dự đoán (predicated measurement)
         *    a_pred = -g_body + ba
         *-------------------------------------------------------------*/
        float a_pred[3] = {
            -g_body[0] + state_.accel_bias[0],
            -g_body[1] + state_.accel_bias[1],
            -g_body[2] + state_.accel_bias[2]
        };

        /*--------------------------------------------------------------
         * 3. Innovation (residual): y = a_meas - a_pred
         *-------------------------------------------------------------*/
        Vec3 y;
        y.d[0][0] = accel[0] - a_pred[0];
        y.d[1][0] = accel[1] - a_pred[1];
        y.d[2][0] = accel[2] - a_pred[2];

        /*--------------------------------------------------------------
         * 4. Measurement Jacobian H (3×9)
         *    H = [-[g_body]×,  0₃ₓ₃,  I₃ₓ₃]
         *
         * Đạo hàm phần tư thế:
         *   ∂h/∂δθ = -[g_body]×
         * Đạo hàm phần accel bias:
         *   ∂h/∂δba = I₃ₓ₃
         *-------------------------------------------------------------*/
        Mat3x9 H;
        H.zero();

        /* H(0:3, 0:3) = -[g_body]× */
        Mat3 skew_g = skew(g_body);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                H.d[i][j] = -skew_g.d[i][j];
            }
        }

        /* H(0:3, 6:9) = I₃ₓ₃ */
        H.d[0][6] = 1.0f;
        H.d[1][7] = 1.0f;
        H.d[2][8] = 1.0f;

        /*--------------------------------------------------------------
         * 5. Innovation covariance: S = H×P×H^T + R_meas (3×3)
         *
         * Scale nhiễu đo lường theo mức độ chuyển động:
         * Càng gần threshold → càng nhiều noise → giảm tin tưởng
         *-------------------------------------------------------------*/
        float accel_var = config_.accel_noise * config_.accel_noise;
        float noise_scale = 1.0f + 10.0f * a_error * a_error;
        accel_var *= noise_scale;

        Mat3 R_meas;
        R_meas.zero();
        R_meas.d[0][0] = accel_var;
        R_meas.d[1][1] = accel_var;
        R_meas.d[2][2] = accel_var;

        Mat3x9 HP   = mat_mul(H, state_.P);          // 3×9
        Mat9x3 HT   = mat_transpose(H);              // 9×3
        Mat3   S    = mat_add(mat_mul(HP, HT), R_meas); // 3×3

        /*--------------------------------------------------------------
         * 6. Kalman gain: K = P × H^T × S^(-1)  (9×3)
         *-------------------------------------------------------------*/
        Mat3   S_inv = mat3_inverse(S);
        Mat9x3 PHT  = mat_mul(state_.P, HT);         // 9×3
        Mat9x3 K    = mat_mul(PHT, S_inv);            // 9×3

        /*--------------------------------------------------------------
         * 7. Error-state correction: δx = K × y  (9×1)
         *-------------------------------------------------------------*/
        Vec9 dx;
        for (int i = 0; i < 9; i++) {
            dx.d[i][0] = K.d[i][0] * y.d[0][0]
                       + K.d[i][1] * y.d[1][0]
                       + K.d[i][2] * y.d[2][0];
        }

        /*--------------------------------------------------------------
         * 8. Injection: error state → nominal state
         *
         *    q  ← q ⊗ Δq(δθ)       Ref: Solà Eq. (282)
         *    bg ← bg + δbg
         *    ba ← ba + δba
         *
         * Δq(δθ) = [cos(‖δθ‖/2), sin(‖δθ‖/2)·δθ/‖δθ‖]
         *-------------------------------------------------------------*/
        float dtheta[3] = {dx.d[0][0], dx.d[1][0], dx.d[2][0]};
        float angle_corr = sqrtf(dtheta[0] * dtheta[0]
                                + dtheta[1] * dtheta[1]
                                + dtheta[2] * dtheta[2]);

        if (angle_corr > 1e-12f) {
            float half_a = angle_corr * 0.5f;
            float s = sinf(half_a) / angle_corr;
            mathlib::Quaternion dq_corr(cosf(half_a),
                                        dtheta[0] * s,
                                        dtheta[1] * s,
                                        dtheta[2] * s);
            state_.q = state_.q * dq_corr;
            state_.q.normalize();
        }

        state_.gyro_bias[0] += dx.d[3][0];
        state_.gyro_bias[1] += dx.d[4][0];
        state_.gyro_bias[2] += dx.d[5][0];

        state_.accel_bias[0] += dx.d[6][0];
        state_.accel_bias[1] += dx.d[7][0];
        state_.accel_bias[2] += dx.d[8][0];

        /*--------------------------------------------------------------
         * 9. Covariance update - Joseph form (ổn định số hơn standard)
         *
         *    P = (I - K×H) × P × (I - K×H)^T + K × R × K^T
         *
         * Ref: Solà Eq. (289)
         *-------------------------------------------------------------*/
        Mat9 KH;
        KH.zero();
        /* KH = K (9×3) × H (3×9) = 9×9 */
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 9; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++) {
                    sum += K.d[i][k] * H.d[k][j];
                }
                KH.d[i][j] = sum;
            }
        }

        Mat9 I_KH;
        I_KH.identity();
        I_KH = mat_sub(I_KH, KH);

        Mat9 I_KH_T = mat_transpose(I_KH);
        Mat9 P_temp = mat_mul(mat_mul(I_KH, state_.P), I_KH_T);

        /* K × R_meas × K^T */
        /* KR = K × diag(accel_var) */
        Mat9x3 KR;
        for (int i = 0; i < 9; i++) {
            KR.d[i][0] = K.d[i][0] * accel_var;
            KR.d[i][1] = K.d[i][1] * accel_var;
            KR.d[i][2] = K.d[i][2] * accel_var;
        }

        /* KR × K^T = (9×3) × (3×9) = 9×9 */
        Mat<3, 9> KT = mat_transpose(K);
        Mat9 KRKT = mat_mul(KR, KT);

        state_.P = mat_add(P_temp, KRKT);

        /* Đảm bảo đối xứng */
        mat_symmetrize(state_.P);
    }
};

} // namespace eskf

#endif // UAV_LIB_ESKF_ESKF_HPP
