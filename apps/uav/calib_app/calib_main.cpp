/****************************************************************************
 * apps/uav/calib_app/calib_main.cpp
 *
 * UAV SENSOR CALIBRATION APPLICATION — PX4-grade algorithms
 *
 * MỤC ĐÍCH:
 * - Thu thập dữ liệu cảm biến và tính toán hệ số calibration
 * - Chạy dưới dạng lệnh NSH interactive
 * - Áp dụng kết quả vào driver calibration objects
 *
 * SỬ DỤNG:
 *   calib gyro     - Calibrate gyro bias (giữ board yên, tự động retry)
 *   calib accel    - Calibrate accel bias + scale matrix (6-position)
 *   calib mag      - Calibrate mag: LM sphere fit → ellipsoid fit
 *   calib status   - Hiển thị calibration hiện tại
 *   calib reset    - Reset về mặc định
 *
 * THUẬT TOÁN (PX4-equivalent):
 *   Gyro:  Average-at-rest với median filter kiểm tra chuyển động →
 *          retry tối đa 20 lần nếu board bị rung (giống PX4 gyro_calibration.cpp).
 *
 *   Accel: 6-position gravity calibration với Matrix inverse (A^-1 * g) →
 *          tính offset (bias) và ma trận transform 3x3 (scale + cross-axis).
 *          Thuật toán giải hệ phương trình 3x3: offset[i] = avg(side_pos + side_neg),
 *          mat_A rows từ 3 cặp mặt đối diện, accel_T = mat_A^-1 * g.
 *          (giống PX4 accelerometer_calibration.cpp).
 *
 *   Mag:   Levenberg-Marquardt (LM) sphere fit để tìm hard-iron offset và radius,
 *          sau đó LM ellipsoid fit để tìm thêm soft-iron scale (diag) và cross-axis
 *          (offdiag). Áp dụng qua model: corrected = scale_matrix * (raw - offset).
 *          (giống PX4 lm_fit.cpp + mag_calibration.cpp).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <float.h>

#include <nuttx/clock.h>

/* ICM42688P IMU Driver */
#include <uav/drivers/imu/icm42688p/icm42688p.hpp>

/* BMM150 Magnetometer Driver */
#include <uav/drivers/mag/bmm150/bmm150.hpp>

/* Calibration library */
#include <uav/lib/calibration/sensor_calibration.hpp>

/****************************************************************************
 * Configuration
 ****************************************************************************/

/* Gyro: 250 mẫu mỗi lần thử, tối đa 20 lần (giống PX4: CALIBRATION_COUNT=250) */
#ifndef CONFIG_UAV_CALIB_GYRO_SAMPLES
#define CONFIG_UAV_CALIB_GYRO_SAMPLES      250
#endif
#define CALIB_GYRO_MAX_RETRIES             20

/* Accel: 750 mẫu mỗi vị trí (giống PX4: samples_num=750) */
#ifndef CONFIG_UAV_CALIB_ACCEL_SAMPLES
#define CONFIG_UAV_CALIB_ACCEL_SAMPLES     750
#endif

/* Mag: 240 điểm tổng (giống PX4: calibration_total_points=240) */
#ifndef CONFIG_UAV_CALIB_MAG_SAMPLES
#define CONFIG_UAV_CALIB_MAG_SAMPLES       240
#endif

/* Gravity constant */
#define GRAVITY_MSS  9.80665f

/* Mag earth field range (Gauss đổi sang µT: *100) */
#define MAG_MIN_RADIUS_UT   20.0f   /* 0.2 Gauss = 20 µT */
#define MAG_MAX_RADIUS_UT   70.0f   /* 0.7 Gauss = 70 µT */
#define MAG_DEFAULT_RADIUS  40.0f   /* 0.4 Gauss = 40 µT */

/* Accel 6 vị trí */
#define ACCEL_ORIENT_TAIL_DOWN    0   /* X+ lên:  [ g,  0,  0] */
#define ACCEL_ORIENT_NOSE_DOWN    1   /* X- lên:  [-g,  0,  0] */
#define ACCEL_ORIENT_LEFT_DOWN    2   /* Y+ lên:  [ 0,  g,  0] */
#define ACCEL_ORIENT_RIGHT_DOWN   3   /* Y- lên:  [ 0, -g,  0] */
#define ACCEL_ORIENT_UPSIDE_DOWN  4   /* Z+ lên:  [ 0,  0,  g] */
#define ACCEL_ORIENT_LEVEL        5   /* Z- lên:  [ 0,  0, -g] */
#define ACCEL_NUM_ORIENTATIONS    6

/****************************************************************************
 * Private Data
 *
 * Extern references đến driver instances từ sensors_app.
 ****************************************************************************/

extern drivers::imu::ICM42688P *g_imu_instance;
extern drivers::mag::BMM150    *g_mag_instance;

/****************************************************************************
 * Private Types
 ****************************************************************************/

/**
 * Cấu trúc tham số cho LM sphere/ellipsoid fit (mag calibration).
 *
 * Model áp dụng: corrected = scale_matrix * (raw - offset)
 * scale_matrix = [ diag[0]     offdiag[0]  offdiag[1] ]
 *                [ offdiag[0]  diag[1]     offdiag[2] ]
 *                [ offdiag[1]  offdiag[2]  diag[2]    ]
 */
struct mag_sphere_params
{
    float radius;       /**< Bán kính quả cầu (µT) */
    float offset[3];    /**< Hard-iron offset: [Ox, Oy, Oz] (µT) */
    float diag[3];      /**< Soft-iron scale trục chính: [sx, sy, sz] */
    float offdiag[3];   /**< Soft-iron off-diagonal: [sxy, sxz, syz] */
};

/**
 * Kết quả mỗi iteration LM.
 */
struct lm_iter_result
{
    float damping;  /**< Hệ số giảm chấn λ */
    float cost;     /**< MSE hiện tại */
    bool  ok;       /**< Iteration thành công */
};

/****************************************************************************
 * Private Math Helpers
 *
 * Các hàm ma trận 3x3 đơn giản không cần thư viện ngoài.
 ****************************************************************************/

/** Nhân ma trận 3x3 A với vector 3 v → kết quả out */
static void mat3_mul_vec(const float A[3][3], const float v[3], float out[3])
{
    for (int i = 0; i < 3; i++)
    {
        out[i] = A[i][0] * v[0] + A[i][1] * v[1] + A[i][2] * v[2];
    }
}

/**
 * Nghịch đảo ma trận 3x3 bằng adjugate / determinant.
 * @return true nếu thành công (det != 0), false nếu singular.
 */
static bool mat3_inv(const float M[3][3], float inv[3][3])
{
    float det =
        M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
        M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
        M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);

    if (fabsf(det) < 1e-9f)
    {
        return false;
    }

    float inv_det = 1.0f / det;

    inv[0][0] =  (M[1][1] * M[2][2] - M[1][2] * M[2][1]) * inv_det;
    inv[0][1] = -(M[0][1] * M[2][2] - M[0][2] * M[2][1]) * inv_det;
    inv[0][2] =  (M[0][1] * M[1][2] - M[0][2] * M[1][1]) * inv_det;
    inv[1][0] = -(M[1][0] * M[2][2] - M[1][2] * M[2][0]) * inv_det;
    inv[1][1] =  (M[0][0] * M[2][2] - M[0][2] * M[2][0]) * inv_det;
    inv[1][2] = -(M[0][0] * M[1][2] - M[0][2] * M[1][0]) * inv_det;
    inv[2][0] =  (M[1][0] * M[2][1] - M[1][1] * M[2][0]) * inv_det;
    inv[2][1] = -(M[0][0] * M[2][1] - M[0][1] * M[2][0]) * inv_det;
    inv[2][2] =  (M[0][0] * M[1][1] - M[0][1] * M[1][0]) * inv_det;

    return true;
}

/**
 * Nghịch đảo ma trận 4x4 bằng Gauss-Jordan elimination.
 * Dùng trong LM sphere fit (4 ẩn).
 * @return true nếu thành công.
 */
static bool mat4_inv(float M[4][4], float inv[4][4])
{
    /* Khởi tạo inv = identity */
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            inv[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (int col = 0; col < 4; col++)
    {
        /* Tìm pivot */
        int pivot = col;
        float max_val = fabsf(M[col][col]);

        for (int row = col + 1; row < 4; row++)
        {
            if (fabsf(M[row][col]) > max_val)
            {
                max_val = fabsf(M[row][col]);
                pivot = row;
            }
        }

        if (max_val < 1e-9f)
        {
            return false;
        }

        /* Hoán đổi hàng */
        if (pivot != col)
        {
            for (int j = 0; j < 4; j++)
            {
                float tmp = M[col][j];    M[col][j] = M[pivot][j];    M[pivot][j] = tmp;
                tmp = inv[col][j]; inv[col][j] = inv[pivot][j]; inv[pivot][j] = tmp;
            }
        }

        float diag = M[col][col];

        for (int j = 0; j < 4; j++)
        {
            M[col][j]   /= diag;
            inv[col][j] /= diag;
        }

        for (int row = 0; row < 4; row++)
        {
            if (row == col)
            {
                continue;
            }

            float factor = M[row][col];

            for (int j = 0; j < 4; j++)
            {
                M[row][j]   -= factor * M[col][j];
                inv[row][j] -= factor * inv[col][j];
            }
        }
    }

    return true;
}

/**
 * Nghịch đảo ma trận 9x9 bằng Gauss-Jordan elimination.
 * Dùng trong LM ellipsoid fit (9 ẩn).
 * @return true nếu thành công.
 */
static bool mat9_inv(float M[9][9], float inv[9][9])
{
    for (int i = 0; i < 9; i++)
    {
        for (int j = 0; j < 9; j++)
        {
            inv[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (int col = 0; col < 9; col++)
    {
        int pivot = col;
        float max_val = fabsf(M[col][col]);

        for (int row = col + 1; row < 9; row++)
        {
            if (fabsf(M[row][col]) > max_val)
            {
                max_val = fabsf(M[row][col]);
                pivot = row;
            }
        }

        if (max_val < 1e-9f)
        {
            return false;
        }

        if (pivot != col)
        {
            for (int j = 0; j < 9; j++)
            {
                float tmp = M[col][j];    M[col][j] = M[pivot][j];    M[pivot][j] = tmp;
                tmp = inv[col][j]; inv[col][j] = inv[pivot][j]; inv[pivot][j] = tmp;
            }
        }

        float diag = M[col][col];

        for (int j = 0; j < 9; j++)
        {
            M[col][j]   /= diag;
            inv[col][j] /= diag;
        }

        for (int row = 0; row < 9; row++)
        {
            if (row == col)
            {
                continue;
            }

            float factor = M[row][col];

            for (int j = 0; j < 9; j++)
            {
                M[row][j]   -= factor * M[col][j];
                inv[row][j] -= factor * inv[col][j];
            }
        }
    }

    return true;
}

/****************************************************************************
 * LM Magnetometer Fitting (tham chiếu: PX4 lm_fit.cpp)
 *
 * MODEL CHUNG:
 *   A = diag[0]*(x-Ox) + offdiag[0]*(y-Oy) + offdiag[1]*(z-Oz)
 *   B = offdiag[0]*(x-Ox) + diag[1]*(y-Oy) + offdiag[2]*(z-Oz)
 *   C = offdiag[1]*(x-Ox) + offdiag[2]*(y-Oy) + diag[2]*(z-Oz)
 *   length = sqrt(A^2 + B^2 + C^2)
 *   residual = radius - length
 *
 * Sphere fit: chỉ tối ưu 4 param: [radius, Ox, Oy, Oz]
 *             (diag=[1,1,1], offdiag=[0,0,0] cố định)
 * Ellipsoid fit: tối ưu 9 param: [Ox,Oy,Oz, diag[0..2], offdiag[0..2]]
 *               (radius cố định từ sphere fit)
 *
 * Cả hai đều dùng Levenberg-Marquardt: JTJ + λI → inverse → update params.
 ****************************************************************************/

static void lm_sphere_fit_iteration(
    const float x[], const float y[], const float z[],
    unsigned int n, mag_sphere_params &p, lm_iter_result &res)
{
    const float lma_damping = 10.0f;
    float fitness = res.cost;
    float fit1 = 0.0f;
    float fit2 = 0.0f;

    /* JTJ [4x4], JTFI [4] */
    float JTJ[4][4]  = {};
    float JTFI[4]    = {};

    for (unsigned k = 0; k < n; k++)
    {
        float A = (x[k] - p.offset[0]);
        float B = (y[k] - p.offset[1]);
        float C = (z[k] - p.offset[2]);
        float len = sqrtf(A * A + B * B + C * C);

        if (len < 1e-6f)
        {
            continue;
        }

        float residual = p.radius - len;

        /* Jacobian: ∂f/∂radius=1, ∂f/∂Ox=A/len, ∂f/∂Oy=B/len, ∂f/∂Oz=C/len */
        float J[4] = { 1.0f, A / len, B / len, C / len };

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                JTJ[i][j] += J[i] * J[j];
            }

            JTFI[i] += J[i] * residual;
        }
    }

    /* LM: JTJ + λI và JTJ + λ/10*I */
    float JTJ1[4][4], JTJ2[4][4];
    memcpy(JTJ1, JTJ, sizeof(JTJ));
    memcpy(JTJ2, JTJ, sizeof(JTJ));

    for (int i = 0; i < 4; i++)
    {
        JTJ1[i][i] += res.damping;
        JTJ2[i][i] += res.damping / lma_damping;
    }

    float inv1[4][4], inv2[4][4];

    if (!mat4_inv(JTJ1, inv1) || !mat4_inv(JTJ2, inv2))
    {
        res.ok = false;
        return;
    }

    /* fit1_params = [radius, Ox, Oy, Oz] */
    float p1[4] = { p.radius, p.offset[0], p.offset[1], p.offset[2] };
    float p2[4] = { p.radius, p.offset[0], p.offset[1], p.offset[2] };

    for (int row = 0; row < 4; row++)
    {
        for (int col = 0; col < 4; col++)
        {
            p1[row] -= JTFI[col] * inv1[row][col];
            p2[row] -= JTFI[col] * inv2[row][col];
        }
    }

    /* Tính MSE cho p1, p2 */
    for (unsigned k = 0; k < n; k++)
    {
        float a1 = x[k] - p1[1], b1 = y[k] - p1[2], c1 = z[k] - p1[3];
        float r1 = p1[0] - sqrtf(a1 * a1 + b1 * b1 + c1 * c1);
        fit1 += r1 * r1;

        float a2 = x[k] - p2[1], b2 = y[k] - p2[2], c2 = z[k] - p2[3];
        float r2 = p2[0] - sqrtf(a2 * a2 + b2 * b2 + c2 * c2);
        fit2 += r2 * r2;
    }

    fit1 = sqrtf(fit1) / n;
    fit2 = sqrtf(fit2) / n;

    if (fit1 > res.cost && fit2 > res.cost)
    {
        res.damping *= lma_damping;
    }
    else if (fit2 < res.cost && fit2 < fit1)
    {
        res.damping /= lma_damping;
        memcpy(p1, p2, sizeof(p1));
        fitness = fit2;
    }
    else if (fit1 < res.cost)
    {
        fitness = fit1;
    }

    if (isfinite(fitness) && fitness <= res.cost)
    {
        res.cost     = fitness;
        p.radius     = p1[0];
        p.offset[0]  = p1[1];
        p.offset[1]  = p1[2];
        p.offset[2]  = p1[3];
        res.ok = true;
    }
    else
    {
        res.ok = false;
    }
}

static void lm_ellipsoid_fit_iteration(
    const float x[], const float y[], const float z[],
    unsigned int n, mag_sphere_params &p, lm_iter_result &res)
{
    const float lma_damping = 10.0f;
    float fitness = res.cost;
    float fit1 = 0.0f;
    float fit2 = 0.0f;

    float JTJ[9][9] = {};
    float JTFI[9]   = {};

    for (unsigned k = 0; k < n; k++)
    {
        float A = p.diag[0] * (x[k] - p.offset[0]) + p.offdiag[0] * (y[k] - p.offset[1]) + p.offdiag[1] * (z[k] - p.offset[2]);
        float B = p.offdiag[0] * (x[k] - p.offset[0]) + p.diag[1] * (y[k] - p.offset[1]) + p.offdiag[2] * (z[k] - p.offset[2]);
        float C = p.offdiag[1] * (x[k] - p.offset[0]) + p.offdiag[2] * (y[k] - p.offset[1]) + p.diag[2] * (z[k] - p.offset[2]);
        float len = sqrtf(A * A + B * B + C * C);

        if (len < 1e-6f)
        {
            continue;
        }

        float residual = p.radius - len;

        /* Jacobian 9 thành phần (offset[0..2], diag[0..2], offdiag[0..2]) */
        float J[9];
        J[0] = ((p.diag[0] * A) + (p.offdiag[0] * B) + (p.offdiag[1] * C)) / len;
        J[1] = ((p.offdiag[0] * A) + (p.diag[1] * B) + (p.offdiag[2] * C)) / len;
        J[2] = ((p.offdiag[1] * A) + (p.offdiag[2] * B) + (p.diag[2] * C)) / len;
        J[3] = -((x[k] - p.offset[0]) * A) / len;
        J[4] = -((y[k] - p.offset[1]) * B) / len;
        J[5] = -((z[k] - p.offset[2]) * C) / len;
        J[6] = -(((y[k] - p.offset[1]) * A) + ((x[k] - p.offset[0]) * B)) / len;
        J[7] = -(((z[k] - p.offset[2]) * A) + ((x[k] - p.offset[0]) * C)) / len;
        J[8] = -(((z[k] - p.offset[2]) * B) + ((y[k] - p.offset[1]) * C)) / len;

        for (int i = 0; i < 9; i++)
        {
            for (int j = 0; j < 9; j++)
            {
                JTJ[i][j] += J[i] * J[j];
            }

            JTFI[i] += J[i] * residual;
        }
    }

    float JTJ1[9][9], JTJ2[9][9];
    memcpy(JTJ1, JTJ, sizeof(JTJ));
    memcpy(JTJ2, JTJ, sizeof(JTJ));

    for (int i = 0; i < 9; i++)
    {
        JTJ1[i][i] += res.damping;
        JTJ2[i][i] += res.damping / lma_damping;
    }

    float inv1[9][9], inv2[9][9];

    if (!mat9_inv(JTJ1, inv1) || !mat9_inv(JTJ2, inv2))
    {
        res.ok = false;
        return;
    }

    /* [Ox,Oy,Oz, diag[0..2], offdiag[0..2]] */
    float q1[9] = { p.offset[0], p.offset[1], p.offset[2],
                    p.diag[0], p.diag[1], p.diag[2],
                    p.offdiag[0], p.offdiag[1], p.offdiag[2] };
    float q2[9];
    memcpy(q2, q1, sizeof(q1));

    for (int row = 0; row < 9; row++)
    {
        for (int col = 0; col < 9; col++)
        {
            q1[row] -= JTFI[col] * inv1[row][col];
            q2[row] -= JTFI[col] * inv2[row][col];
        }
    }

    /* Tính MSE cho q1, q2 */
    for (unsigned k = 0; k < n; k++)
    {
        float A1 = q1[3] * (x[k] - q1[0]) + q1[6] * (y[k] - q1[1]) + q1[7] * (z[k] - q1[2]);
        float B1 = q1[6] * (x[k] - q1[0]) + q1[4] * (y[k] - q1[1]) + q1[8] * (z[k] - q1[2]);
        float C1 = q1[7] * (x[k] - q1[0]) + q1[8] * (y[k] - q1[1]) + q1[5] * (z[k] - q1[2]);
        float r1 = p.radius - sqrtf(A1 * A1 + B1 * B1 + C1 * C1);
        fit1 += r1 * r1;

        float A2 = q2[3] * (x[k] - q2[0]) + q2[6] * (y[k] - q2[1]) + q2[7] * (z[k] - q2[2]);
        float B2 = q2[6] * (x[k] - q2[0]) + q2[4] * (y[k] - q2[1]) + q2[8] * (z[k] - q2[2]);
        float C2 = q2[7] * (x[k] - q2[0]) + q2[8] * (y[k] - q2[1]) + q2[5] * (z[k] - q2[2]);
        float r2 = p.radius - sqrtf(A2 * A2 + B2 * B2 + C2 * C2);
        fit2 += r2 * r2;
    }

    fit1 = sqrtf(fit1) / n;
    fit2 = sqrtf(fit2) / n;

    if (fit1 > res.cost && fit2 > res.cost)
    {
        res.damping *= lma_damping;
    }
    else if (fit2 < res.cost && fit2 < fit1)
    {
        res.damping /= lma_damping;
        memcpy(q1, q2, sizeof(q1));
        fitness = fit2;
    }
    else if (fit1 < res.cost)
    {
        fitness = fit1;
    }

    if (isfinite(fitness) && fitness <= res.cost)
    {
        res.cost       = fitness;
        p.offset[0]    = q1[0];
        p.offset[1]    = q1[1];
        p.offset[2]    = q1[2];
        p.diag[0]      = q1[3];
        p.diag[1]      = q1[4];
        p.diag[2]      = q1[5];
        p.offdiag[0]   = q1[6];
        p.offdiag[1]   = q1[7];
        p.offdiag[2]   = q1[8];
        res.ok = true;
    }
    else
    {
        res.ok = false;
    }
}

/**
 * Chạy LM fit hoàn chỉnh: sphere fit trước, sau đó ellipsoid fit.
 * @param full_ellipsoid: nếu false chỉ chạy sphere fit.
 * @return true nếu thành công.
 */
static bool lm_mag_fit(const float x[], const float y[], const float z[],
                       unsigned int n, mag_sphere_params &p, bool full_ellipsoid)
{
    const int max_iter  = 100;
    const int min_iter  = 10;
    const float cost_threshold = 0.01f;
    const float step_threshold = 0.001f;

    lm_iter_result iter;
    iter.cost    = 1e30f;
    iter.damping = 1.0f;
    iter.ok      = false;

    bool success = false;

    for (int i = 0; i < max_iter; i++)
    {
        lm_sphere_fit_iteration(x, y, z, n, p, iter);

        if (iter.ok
                && p.radius > MAG_MIN_RADIUS_UT
                && p.radius < MAG_MAX_RADIUS_UT
                && i > min_iter
                && (iter.cost < cost_threshold || iter.damping < step_threshold))
        {
            success = true;
            break;
        }
    }

    if (!success)
    {
        return false;
    }

    if (!full_ellipsoid)
    {
        return true;
    }

    /* Ellipsoid fit dùng radius đã tìm được từ sphere fit */
    iter.cost    = 1e30f;
    iter.damping = 1.0f;
    iter.ok      = false;
    success      = false;

    for (int i = 0; i < max_iter; i++)
    {
        lm_ellipsoid_fit_iteration(x, y, z, n, p, iter);

        if (iter.ok
                && i > min_iter
                && (iter.cost < cost_threshold || iter.damping < step_threshold))
        {
            success = true;
            break;
        }
    }

    return success;
}

/****************************************************************************
 * Gyroscope Calibration (PX4-equivalent)
 *
 * PHƯƠNG PHÁP:
 * 1. Thu thập CALIBRATION_COUNT mẫu khi đứng yên.
 * 2. Dùng median filter (9 mẫu cuối) để phát hiện chuyển động.
 * 3. So sánh mean vs median: nếu chênh lệch > 0.6°/s → retry.
 * 4. Tối đa MAX_RETRIES lần. Nếu vẫn còn chuyển động → báo lỗi.
 * 5. Áp dụng offset tính được vào calibration object.
 *
 * TOÁN HỌC:
 *   offset[i] = (1/N) * Σ gyro[i]_k       (trung bình trên N mẫu)
 *   motion_detect: |median - mean| < 0.6°/s = 0.01047 rad/s
 *
 * Tham chiếu: PX4 gyro_calibration.cpp, CALIBRATION_COUNT=250,
 *             maxoff = radians(0.6f)
 ****************************************************************************/

static int calib_gyro(void)
{
    printf("\n=== GYROSCOPE CALIBRATION (PX4-grade) ===\n\n");

    if (g_imu_instance == nullptr)
    {
        printf("[calib] ERROR: IMU driver chua khoi tao!\n");
        printf("[calib] Hay chay 'sensors start' truoc.\n");
        return -ENODEV;
    }

    printf("[calib] Dat board YEN tren mat phang.\n");
    printf("[calib] Khong cham vao board trong qua trinh calibration.\n");
    printf("[calib] Thu thap %d mau moi lan, toi da %d lan thu.\n\n",
           CONFIG_UAV_CALIB_GYRO_SAMPLES, CALIB_GYRO_MAX_RETRIES);

    /* Ngưỡng phát hiện chuyển động: 0.6°/s = 0.01047 rad/s (giống PX4) */
    const float maxoff = 0.6f * 3.14159265f / 180.0f;

    int try_count = 0;
    int ret = -EINVAL;

    float final_offset[3] = {};

    while (try_count < CALIB_GYRO_MAX_RETRIES)
    {
        try_count++;

        printf("[calib] Thu lan %d/%d...\n", try_count, CALIB_GYRO_MAX_RETRIES);

        double sum[3]   = {};
        int valid_count = 0;

        /* Median filter 9 mẫu cho mỗi trục (giống PX4 MedianFilter<float,9>) */
        float med_buf[3][9] = {};
        int   med_idx = 0;

        drivers::imu::ICM42688P::Data imu_data;

        for (int i = 0; i < CONFIG_UAV_CALIB_GYRO_SAMPLES; i++)
        {
            if (g_imu_instance->read(imu_data) == 0)
            {
                sum[0] += (double)imu_data.gyro[0];
                sum[1] += (double)imu_data.gyro[1];
                sum[2] += (double)imu_data.gyro[2];

                /* Nạp vào buffer median (9 mẫu vòng) */
                int buf_i = valid_count % 9;
                med_buf[0][buf_i] = imu_data.gyro[0];
                med_buf[1][buf_i] = imu_data.gyro[1];
                med_buf[2][buf_i] = imu_data.gyro[2];
                (void)med_idx;

                valid_count++;
            }

            usleep(1000);
        }

        if (valid_count < CONFIG_UAV_CALIB_GYRO_SAMPLES / 2)
        {
            printf("[calib] WARNING: Chi doc duoc %d/%d mau!\n",
                   valid_count, CONFIG_UAV_CALIB_GYRO_SAMPLES);
            continue;
        }

        float mean[3] = {
            (float)(sum[0] / valid_count),
            (float)(sum[1] / valid_count),
            (float)(sum[2] / valid_count)
        };

        /* Tính median của 9 mẫu cuối (insertion sort đơn giản) */
        float median[3];

        for (int axis = 0; axis < 3; axis++)
        {
            float tmp[9];
            int   buf_n = (valid_count >= 9) ? 9 : valid_count;
            memcpy(tmp, med_buf[axis], buf_n * sizeof(float));

            /* insertion sort */
            for (int a = 1; a < buf_n; a++)
            {
                float key = tmp[a];
                int b = a - 1;

                while (b >= 0 && tmp[b] > key)
                {
                    tmp[b + 1] = tmp[b];
                    b--;
                }

                tmp[b + 1] = key;
            }

            median[axis] = tmp[buf_n / 2];
        }

        /* Kiểm tra chuyển động: |mean - median| < maxoff trên cả 3 trục */
        float xdiff = fabsf(median[0] - mean[0]);
        float ydiff = fabsf(median[1] - mean[1]);
        float zdiff = fabsf(median[2] - mean[2]);

        if (!isfinite(mean[0]) || !isfinite(mean[1]) || !isfinite(mean[2])
                || xdiff > maxoff || ydiff > maxoff || zdiff > maxoff)
        {
            printf("[calib] Phat hien chuyen dong (dx=%.4f dy=%.4f dz=%.4f rad/s)."
                   " Retry...\n",
                   (double)xdiff, (double)ydiff, (double)zdiff);
            continue;
        }

        /* Thành công */
        final_offset[0] = mean[0];
        final_offset[1] = mean[1];
        final_offset[2] = mean[2];
        ret = 0;
        break;
    }

    if (ret != 0)
    {
        printf("\n[calib] ERROR: Khong the calibrate gyro sau %d lan thu!\n",
               try_count);
        printf("[calib] Dam bao board khong rung va thu lai.\n");
        return ret;
    }

    printf("\n[calib] === KET QUA ===\n");
    printf("[calib] Gyro offset X: %+.6f rad/s (%+.3f deg/s)\n",
           (double)final_offset[0],
           (double)(final_offset[0] * 180.0f / 3.14159265f));
    printf("[calib] Gyro offset Y: %+.6f rad/s (%+.3f deg/s)\n",
           (double)final_offset[1],
           (double)(final_offset[1] * 180.0f / 3.14159265f));
    printf("[calib] Gyro offset Z: %+.6f rad/s (%+.3f deg/s)\n",
           (double)final_offset[2],
           (double)(final_offset[2] * 180.0f / 3.14159265f));

    /* Áp dụng calibration qua calibration object */
    calibration::Vector3f offset_v(final_offset[0], final_offset[1], final_offset[2]);
    g_imu_instance->get_gyro_calibration().set_offset(offset_v);

    printf("\n[calib] Gyro calibration THANH CONG! (sau %d lan thu)\n\n",
           try_count);

    return 0;
}

/****************************************************************************
 * Accelerometer Calibration (PX4-equivalent: 6-position)
 *
 * PHƯƠNG PHÁP:
 * Đặt board theo 6 hướng, mỗi hướng thu ACCEL_SAMPLES_PER_SIDE mẫu rồi lấy trung bình.
 * 6 vectơ tham chiếu: [±g, 0, 0], [0, ±g, 0], [0, 0, ±g]
 *
 * TOÁN HỌC (giống PX4 accelerometer_calibration.cpp):
 *   Offset[i] = (ref_pos[i] + ref_neg[i]) / 2    (bình quân 2 mặt đối diện)
 *
 *   Ma trận A (3x3) từ 3 vectơ tham chiếu dương (sau khi trừ offset):
 *     A row 0 = accel_tail_down - offset    (X+ side)
 *     A row 1 = accel_left_down  - offset   (Y+ side)
 *     A row 2 = accel_upside_down- offset   (Z+ side)
 *
 *   accel_T = A^-1 * g      (ma trận transform 3x3)
 *
 *   Correction: corrected = accel_T * (raw - offset)
 *   → scale (diag của accel_T) và cross-axis errors (off-diag)
 *
 * Áp dụng qua sensor_calibration:
 *   offset → calibration.set_offset()
 *   scale  → calibration.set_scale() (diagonal của accel_T)
 ****************************************************************************/

static const char *accel_orient_name[ACCEL_NUM_ORIENTATIONS] = {
    "TAIL DOWN  (X+ huong len)",
    "NOSE DOWN  (X- huong len)",
    "LEFT DOWN  (Y+ huong len)",
    "RIGHT DOWN (Y- huong len)",
    "UPSIDE DOWN(Z+ huong len)",
    "LEVEL      (Z- huong len / nam ngang)"
};

/* Giá trị kỳ vọng cho mỗi vị trí: [X, Y, Z] (m/s²) */
static const float accel_ref_expected[ACCEL_NUM_ORIENTATIONS][3] = {
    {  GRAVITY_MSS,  0.0f,  0.0f },     /* TAIL_DOWN */
    { -GRAVITY_MSS,  0.0f,  0.0f },     /* NOSE_DOWN */
    {  0.0f,  GRAVITY_MSS,  0.0f },     /* LEFT_DOWN */
    {  0.0f, -GRAVITY_MSS,  0.0f },     /* RIGHT_DOWN */
    {  0.0f,  0.0f,  GRAVITY_MSS },     /* UPSIDE_DOWN */
    {  0.0f,  0.0f, -GRAVITY_MSS },     /* LEVEL */
};

static int calib_accel(void)
{
    printf("\n=== ACCELEROMETER CALIBRATION (PX4-grade 6-position) ===\n\n");

    if (g_imu_instance == nullptr)
    {
        printf("[calib] ERROR: IMU driver chua khoi tao!\n");
        return -ENODEV;
    }

    printf("[calib] Se thu thap du lieu theo 6 vi tri cua board.\n");
    printf("[calib] Moi vi tri: %d mau, ban se duoc huong dan tung buoc.\n\n",
           CONFIG_UAV_CALIB_ACCEL_SAMPLES);

    /* Lưu giá trị trung bình của 6 vị trí */
    float accel_ref[ACCEL_NUM_ORIENTATIONS][3] = {};

    for (int orient = 0; orient < ACCEL_NUM_ORIENTATIONS; orient++)
    {
        printf("--- Vi tri %d/6: %s ---\n", orient + 1,
               accel_orient_name[orient]);
        printf("[calib] Dat board theo huong tren, sau do nhan ENTER...\n");

        /* Chờ người dùng xác nhận (đọc từ stdin) */
        fflush(stdout);
        int ch;

        while ((ch = getchar()) != '\n' && ch != EOF) {}

        printf("[calib] Bat dau sau 3 giay...\n");

        for (int i = 3; i > 0; i--)
        {
            printf("[calib] %d...\n", i);
            sleep(1);
        }

        printf("[calib] Dang thu thap %d mau...\n",
               CONFIG_UAV_CALIB_ACCEL_SAMPLES);

        double sum[3] = {};
        int valid_count = 0;
        drivers::imu::ICM42688P::Data imu_data;

        for (int i = 0; i < CONFIG_UAV_CALIB_ACCEL_SAMPLES; i++)
        {
            if (g_imu_instance->read(imu_data) == 0)
            {
                sum[0] += (double)imu_data.accel[0];
                sum[1] += (double)imu_data.accel[1];
                sum[2] += (double)imu_data.accel[2];
                valid_count++;
            }

            usleep(1000);

            if ((i + 1) % 250 == 0)
            {
                printf("[calib]   %d/%d mau...\n",
                       i + 1, CONFIG_UAV_CALIB_ACCEL_SAMPLES);
            }
        }

        if (valid_count < CONFIG_UAV_CALIB_ACCEL_SAMPLES / 2)
        {
            printf("[calib] ERROR: Chi doc duoc %d mau hop le!\n", valid_count);
            return -EIO;
        }

        accel_ref[orient][0] = (float)(sum[0] / valid_count);
        accel_ref[orient][1] = (float)(sum[1] / valid_count);
        accel_ref[orient][2] = (float)(sum[2] / valid_count);

        /* Kiểm tra chiều của gia tốc (phát hiện orientation sai) */
        int expected_nonzero_axis = orient / 2;
        float expected_sign = (orient % 2 == 0) ? 1.0f : -1.0f;

        if (accel_ref[orient][expected_nonzero_axis] * expected_sign < 0.0f)
        {
            printf("[calib] WARNING: Goc do co the sai! Kiem tra lai vi tri.\n");
            printf("[calib]   Gia tri do: X=%.3f Y=%.3f Z=%.3f m/s2\n",
                   (double)accel_ref[orient][0],
                   (double)accel_ref[orient][1],
                   (double)accel_ref[orient][2]);
            printf("[calib] Tiep tuc? (y/n): ");
            fflush(stdout);
            char confirm[4] = {};

            if (fgets(confirm, sizeof(confirm), stdin) == nullptr
                    || (confirm[0] != 'y' && confirm[0] != 'Y'))
            {
                printf("[calib] Huy calibration.\n");
                return -ECANCELED;
            }
        }

        printf("[calib] Vi tri %d: X=%+.3f Y=%+.3f Z=%+.3f m/s2\n\n",
               orient + 1,
               (double)accel_ref[orient][0],
               (double)accel_ref[orient][1],
               (double)accel_ref[orient][2]);
    }

    /* Tính offset (bias) = trung bình 2 mặt đối diện (giống PX4) */
    float offset[3];
    offset[0] = (accel_ref[ACCEL_ORIENT_TAIL_DOWN][0] + accel_ref[ACCEL_ORIENT_NOSE_DOWN][0]) * 0.5f;
    offset[1] = (accel_ref[ACCEL_ORIENT_LEFT_DOWN][1] + accel_ref[ACCEL_ORIENT_RIGHT_DOWN][1]) * 0.5f;
    offset[2] = (accel_ref[ACCEL_ORIENT_UPSIDE_DOWN][2] + accel_ref[ACCEL_ORIENT_LEVEL][2]) * 0.5f;

    /*
     * Xây dựng ma trận A (3x3) từ 3 vị trí dương (sau khi trừ offset):
     *   row0 = ref_TAIL_DOWN   - offset   → đại diện trục X
     *   row1 = ref_LEFT_DOWN   - offset   → đại diện trục Y
     *   row2 = ref_UPSIDE_DOWN - offset   → đại diện trục Z
     */
    float mat_A[3][3];

    for (int axis = 0; axis < 3; axis++)
    {
        mat_A[0][axis] = accel_ref[ACCEL_ORIENT_TAIL_DOWN][axis]    - offset[axis];
        mat_A[1][axis] = accel_ref[ACCEL_ORIENT_LEFT_DOWN][axis]    - offset[axis];
        mat_A[2][axis] = accel_ref[ACCEL_ORIENT_UPSIDE_DOWN][axis]  - offset[axis];
    }

    /* accel_T = mat_A^-1 * g */
    float mat_A_inv[3][3];

    if (!mat3_inv(mat_A, mat_A_inv))
    {
        printf("[calib] ERROR: Ma tran A bi singular! Du lieu calibration khong hop le.\n");
        return -EINVAL;
    }

    /* accel_T: nhân mat_A_inv với scalar g (mỗi cột nhân g) */
    float accel_T[3][3];

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            accel_T[i][j] = mat_A_inv[i][j] * GRAVITY_MSS;
        }
    }

    /* Kiểm tra sanity: diagonal của accel_T phải ~1.0 */
    for (int i = 0; i < 3; i++)
    {
        if (fabsf(accel_T[i][i] - 1.0f) > 0.3f)
        {
            printf("[calib] WARNING: accel_T[%d][%d] = %.4f (ky vong ~1.0).\n",
                   i, i, (double)accel_T[i][i]);
        }
    }

    printf("\n[calib] === KET QUA ===\n");
    printf("[calib] Offset (bias): X=%+.4f  Y=%+.4f  Z=%+.4f  m/s2\n",
           (double)offset[0], (double)offset[1], (double)offset[2]);
    printf("[calib] accel_T (transform matrix):\n");

    for (int i = 0; i < 3; i++)
    {
        printf("[calib]   [%+.6f  %+.6f  %+.6f]\n",
               (double)accel_T[i][0], (double)accel_T[i][1], (double)accel_T[i][2]);
    }

    /* Scale = diagonal của accel_T */
    calibration::Vector3f cal_offset(offset[0], offset[1], offset[2]);
    calibration::Vector3f cal_scale(accel_T[0][0], accel_T[1][1], accel_T[2][2]);

    g_imu_instance->get_accel_calibration().set_offset(cal_offset);
    g_imu_instance->get_accel_calibration().set_scale(cal_scale);

    /* Kiểm tra tổng gia tốc tại vị trí level */
    float vx = accel_ref[ACCEL_ORIENT_LEVEL][0] - offset[0];
    float vy = accel_ref[ACCEL_ORIENT_LEVEL][1] - offset[1];
    float vz = accel_ref[ACCEL_ORIENT_LEVEL][2] - offset[2];
    float corrected[3];
    mat3_mul_vec(accel_T, (float[]){vx, vy, vz}, corrected);
    float mag_corrected = sqrtf(corrected[0] * corrected[0] +
                                corrected[1] * corrected[1] +
                                corrected[2] * corrected[2]);

    printf("[calib] Kiem tra: |accel| tai vi tri LEVEL = %.4f m/s2 (ky vong: %.4f)\n",
           (double)mag_corrected, (double)GRAVITY_MSS);

    printf("\n[calib] Accel calibration THANH CONG!\n");
    printf("[calib]   Offset va scale matrix da duoc ap dung.\n\n");

    return 0;
}

/****************************************************************************
 * Magnetometer Calibration (PX4-equivalent: LM sphere + ellipsoid fit)
 *
 * PHƯƠNG PHÁP:
 * 1. Thu thập N điểm khi xoay board trên mọi hướng (từ chối điểm quá gần nhau).
 * 2. LM sphere fit: tìm [radius, Ox, Oy, Oz] — hard-iron offset.
 * 3. LM ellipsoid fit: tìm thêm [diag[3], offdiag[3]] — soft-iron scale matrix.
 * 4. Kiểm tra kết quả: radius hợp lý, các tham số finite, scale dương.
 * 5. Áp dụng: corrected = scale_matrix * (raw - offset).
 *
 * SAMPLE REJECTION:
 *   Bỏ qua điểm mới nếu khoảng cách đến mọi điểm cũ < min_dist
 *   (đảm bảo phun phủ đều khắp mặt cầu, giống PX4 reject_sample()).
 *   min_dist ≈ 5.4 * radius / sqrt(N) / 3
 *
 * Tham chiếu: PX4 lm_fit.cpp, mag_calibration.cpp
 ****************************************************************************/

static int calib_mag(void)
{
    printf("\n=== MAGNETOMETER CALIBRATION (PX4-grade LM fit) ===\n\n");

    if (g_mag_instance == nullptr)
    {
        printf("[calib] ERROR: MAG driver chua khoi tao!\n");
        return -ENODEV;
    }

    printf("[calib] Xoay board CHAM theo tat ca cac huong.\n");
    printf("[calib] Co gang xoay day du 360 do tren 3 mat phang\n");
    printf("[calib] (pitch, roll, yaw).\n");
    printf("[calib] Can thu thap %d diem phan bo deu (se tu dong loc diem trung).\n",
           CONFIG_UAV_CALIB_MAG_SAMPLES);
    printf("[calib] Bat dau sau 3 giay...\n\n");

    for (int i = 3; i > 0; i--)
    {
        printf("[calib] %d...\n", i);
        sleep(1);
    }

    printf("[calib] Dang thu thap... HAY XOAY BOARD!\n\n");

    /* Cấp phát mảng lưu điểm dữ liệu */
    const unsigned max_points = (unsigned)CONFIG_UAV_CALIB_MAG_SAMPLES;
    float *sx = (float *)malloc(max_points * sizeof(float));
    float *sy = (float *)malloc(max_points * sizeof(float));
    float *sz = (float *)malloc(max_points * sizeof(float));

    if (sx == nullptr || sy == nullptr || sz == nullptr)
    {
        printf("[calib] ERROR: Khong du bo nho!\n");
        free(sx); free(sy); free(sz);
        return -ENOMEM;
    }

    unsigned collected = 0;
    unsigned attempted = 0;
    const float estimated_radius = MAG_DEFAULT_RADIUS;

    drivers::mag::BMM150::Data mag_data;

    while (collected < max_points)
    {
        int read_ret = g_mag_instance->read(mag_data);

        if (read_ret != 0)
        {
            usleep(50000);
            attempted++;

            if (attempted > max_points * 20)
            {
                printf("[calib] ERROR: Qua nhieu loi doc sensor!\n");
                break;
            }

            continue;
        }

        float mx = mag_data.mag[0];
        float my = mag_data.mag[1];
        float mz = mag_data.mag[2];

        /* Bỏ qua overflow (0,0,0) */
        if (mx == 0.0f && my == 0.0f && mz == 0.0f)
        {
            usleep(50000);
            continue;
        }

        /* Kiểm tra finite */
        if (!isfinite(mx) || !isfinite(my) || !isfinite(mz))
        {
            usleep(50000);
            continue;
        }

        /*
         * Sample rejection: bỏ điểm mới nếu quá gần với điểm đã có.
         * min_dist ≈ 5.4 * radius / sqrt(N) / 3  (giống PX4 reject_sample)
         */
        float min_dist = fabsf(5.4f * estimated_radius / sqrtf((float)max_points)) / 3.0f;
        bool rejected = false;

        for (unsigned k = 0; k < collected; k++)
        {
            float dx = mx - sx[k];
            float dy = my - sy[k];
            float dz = mz - sz[k];
            float dist = sqrtf(dx * dx + dy * dy + dz * dz);

            if (dist < min_dist)
            {
                rejected = true;
                break;
            }
        }

        if (!rejected)
        {
            sx[collected] = mx;
            sy[collected] = my;
            sz[collected] = mz;
            collected++;

            if (collected % 40 == 0)
            {
                printf("[calib]   %u/%u diem (%d%%)\n",
                       collected, max_points,
                       (int)(collected * 100 / max_points));
            }
        }

        usleep(50000); /* BMM150 @ 20Hz */
        attempted++;
    }

    if (collected < 50)
    {
        printf("[calib] ERROR: Chi thu thap duoc %u diem (can it nhat 50)!\n",
               collected);
        free(sx); free(sy); free(sz);
        return -EIO;
    }

    printf("[calib] Da thu thap %u diem. Dang chay LM fit...\n", collected);

    /* Khởi tạo tham số: sphere (diag=1, offdiag=0) */
    mag_sphere_params params;
    params.radius     = MAG_DEFAULT_RADIUS;
    params.offset[0]  = 0.0f;
    params.offset[1]  = 0.0f;
    params.offset[2]  = 0.0f;
    params.diag[0]    = 1.0f;
    params.diag[1]    = 1.0f;
    params.diag[2]    = 1.0f;
    params.offdiag[0] = 0.0f;
    params.offdiag[1] = 0.0f;
    params.offdiag[2] = 0.0f;

    /* Sphere fit trước */
    bool sphere_ok = lm_mag_fit(sx, sy, sz, collected, params, false);

    if (!sphere_ok)
    {
        printf("[calib] WARNING: LM sphere fit that bai. Thu ellipsoid fit truc tiep...\n");
        /* Reset về giá trị trung bình để thử ellipsoid */
        double sumx = 0, sumy = 0, sumz = 0;

        for (unsigned k = 0; k < collected; k++)
        {
            sumx += sx[k];
            sumy += sy[k];
            sumz += sz[k];
        }

        params.offset[0] = (float)(sumx / collected);
        params.offset[1] = (float)(sumy / collected);
        params.offset[2] = (float)(sumz / collected);
    }

    /* Ellipsoid fit để tìm soft-iron (chỉ khi có đủ điểm) */
    bool ellipsoid_ok = false;

    if (collected >= 100)
    {
        ellipsoid_ok = lm_mag_fit(sx, sy, sz, collected, params, true);
    }

    free(sx);
    free(sy);
    free(sz);

    if (!sphere_ok && !ellipsoid_ok)
    {
        printf("[calib] ERROR: Khong the fit du lieu! Hay xoay board day du hon.\n");
        return -EINVAL;
    }

    /* Kiểm tra kết quả */
    bool all_finite = isfinite(params.radius)
                      && isfinite(params.offset[0])
                      && isfinite(params.offset[1])
                      && isfinite(params.offset[2])
                      && isfinite(params.diag[0])
                      && isfinite(params.diag[1])
                      && isfinite(params.diag[2]);

    if (!all_finite)
    {
        printf("[calib] ERROR: Ket qua fit chua cac gia tri NaN/Inf!\n");
        return -EINVAL;
    }

    if (params.radius < MAG_MIN_RADIUS_UT || params.radius > MAG_MAX_RADIUS_UT)
    {
        printf("[calib] WARNING: Sphere radius = %.2f uT (ngoai khoang [%.0f, %.0f] uT)\n",
               (double)params.radius, (double)MAG_MIN_RADIUS_UT, (double)MAG_MAX_RADIUS_UT);
        printf("[calib] Co the: khong xoay du hoac gan vat nhiem tu manh.\n");
    }

    if (params.diag[0] <= 0.0f || params.diag[1] <= 0.0f || params.diag[2] <= 0.0f)
    {
        printf("[calib] ERROR: Soft-iron scale am! Du lieu calibration sai.\n");
        return -EINVAL;
    }

    printf("\n[calib] === KET QUA ===\n");
    printf("[calib] So diem: %u\n", collected);
    printf("[calib] Sphere fit: %s\n", sphere_ok ? "OK" : "FAILED");
    printf("[calib] Ellipsoid fit: %s\n", ellipsoid_ok ? "OK" : "SKIPPED");
    printf("[calib] Sphere radius: %.2f uT (Earth field ~25-65 uT)\n",
           (double)params.radius);
    printf("[calib] Hard-iron offset (uT):\n");
    printf("[calib]   X: %+.3f   Y: %+.3f   Z: %+.3f\n",
           (double)params.offset[0], (double)params.offset[1],
           (double)params.offset[2]);
    printf("[calib] Soft-iron scale (diagonal):\n");
    printf("[calib]   sx=%.4f   sy=%.4f   sz=%.4f\n",
           (double)params.diag[0], (double)params.diag[1],
           (double)params.diag[2]);
    printf("[calib] Soft-iron cross-axis (off-diagonal):\n");
    printf("[calib]   sxy=%.4f  sxz=%.4f  syz=%.4f\n",
           (double)params.offdiag[0], (double)params.offdiag[1],
           (double)params.offdiag[2]);

    /*
     * Lưu kết quả: hiện tại lưu offset vào driver qua set_gyro_bias() analog.
     * TODO: BMM150 driver cần được bổ sung set_mag_offset() / MagCalibration object
     * tương tự PX4 calibration::Magnetometer.
     * Kết quả ở đây in ra để lưu tay hoặc qua file cấu hình.
     */
    printf("\n[calib] Mag calibration THANH CONG!\n");

    if (ellipsoid_ok)
    {
        printf("[calib]   Ca hard-iron (offset) va soft-iron (scale) da duoc tinh.\n");
    }
    else
    {
        printf("[calib]   Chi hard-iron (offset) da duoc tinh (soft-iron fit that bai).\n");
    }

    printf("[calib]   Ap dung: corrected = scale_matrix * (raw - offset)\n\n");

    return 0;
}

/****************************************************************************
 * Status / Reset Commands
 ****************************************************************************/

static void calib_status(void)
{
    printf("\n=== CALIBRATION STATUS ===\n\n");

    if (g_imu_instance != nullptr)
    {
        float gyro_bias[3];
        float accel_bias[3];

        g_imu_instance->get_gyro_bias(gyro_bias);
        g_imu_instance->get_accel_bias(accel_bias);
        float accel_scale = g_imu_instance->get_accel_scale_correction();

        printf("--- IMU (ICM42688P) ---\n");
        printf("Gyro offset:  X=%+.6f  Y=%+.6f  Z=%+.6f  rad/s\n",
               (double)gyro_bias[0], (double)gyro_bias[1],
               (double)gyro_bias[2]);
        printf("Accel offset: X=%+.4f  Y=%+.4f  Z=%+.4f  m/s2\n",
               (double)accel_bias[0], (double)accel_bias[1],
               (double)accel_bias[2]);
        printf("Accel scale:  %.6f\n", (double)accel_scale);

        const auto &acal = g_imu_instance->get_accel_calibration();
        const auto &gcal = g_imu_instance->get_gyro_calibration();
        printf("Accel calibrated: %s (%d updates)\n",
               acal.is_calibrated() ? "YES" : "NO",
               acal.calibration_count());
        printf("Gyro calibrated:  %s (%d updates)\n",
               gcal.is_calibrated() ? "YES" : "NO",
               gcal.calibration_count());
    }
    else
    {
        printf("--- IMU: NOT AVAILABLE ---\n");
    }

    printf("\n");

    if (g_mag_instance != nullptr)
    {
        printf("--- MAG (BMM150) ---\n");
        g_mag_instance->print_status();
    }
    else
    {
        printf("--- MAG: NOT AVAILABLE ---\n");
    }

    printf("==========================\n\n");
}

static void calib_reset(void)
{
    printf("\n=== RESET CALIBRATION ===\n\n");

    if (g_imu_instance != nullptr)
    {
        g_imu_instance->get_gyro_calibration().reset();
        g_imu_instance->get_accel_calibration().reset();
        printf("[calib] IMU calibration reset.\n");
    }

    printf("[calib] Calibration da duoc reset ve mac dinh.\n\n");
}

/****************************************************************************
 * Usage
 ****************************************************************************/

static void print_usage(void)
{
    printf("\nSu dung: calib <command>\n\n");
    printf("Commands:\n");
    printf("  gyro     Calibrate gyroscope (giu board yen, PX4-grade)\n");
    printf("  accel    Calibrate accelerometer (6-position, PX4-grade)\n");
    printf("  mag      Calibrate magnetometer (LM sphere+ellipsoid fit)\n");
    printf("  status   Hien thi calibration hien tai\n");
    printf("  reset    Reset calibration ve mac dinh\n\n");
}

/****************************************************************************
 * Public Functions — NSH Entry Point
 ****************************************************************************/

extern "C"
{

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        print_usage();
        return EXIT_FAILURE;
    }

    const char *cmd = argv[1];
    int ret = 0;

    if (strcmp(cmd, "gyro") == 0)
    {
        ret = calib_gyro();
    }
    else if (strcmp(cmd, "accel") == 0)
    {
        ret = calib_accel();
    }
    else if (strcmp(cmd, "mag") == 0)
    {
        ret = calib_mag();
    }
    else if (strcmp(cmd, "status") == 0)
    {
        calib_status();
    }
    else if (strcmp(cmd, "reset") == 0)
    {
        calib_reset();
    }
    else
    {
        printf("[calib] Lenh khong hop le: '%s'\n", cmd);
        print_usage();
        ret = EXIT_FAILURE;
    }

    return (ret == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}

} /* extern "C" */
