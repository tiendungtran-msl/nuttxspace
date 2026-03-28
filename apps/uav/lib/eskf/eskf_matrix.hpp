/****************************************************************************
 * apps/uav/lib/eskf/eskf_matrix.hpp
 *
 * Template Matrix Utilities cho ESKF
 *
 * MỤC ĐÍCH:
 * - Cung cấp phép tính ma trận nhỏ (3×3, 9×9, 3×9, 9×3)
 * - Tối ưu cho embedded: static allocation, no dynamic memory
 * - Template cho phép compiler tối ưu hóa kích thước cố định
 *
 * SỬ DỤNG:
 * - Mat<R,C>: Ma trận R×C
 * - mat_mul(A, B): Nhân ma trận (kích thước tự suy luận)
 * - mat_transpose(A): Chuyển vị
 * - mat3_inverse(A): Nghịch đảo 3×3 (công thức giải tích)
 *
 ****************************************************************************/

#ifndef UAV_LIB_ESKF_MATRIX_HPP
#define UAV_LIB_ESKF_MATRIX_HPP

#include <cstring>
#include <cmath>

namespace eskf {

/****************************************************************************
 * Template Matrix Class
 ****************************************************************************/

template<int R, int C>
struct Mat {
    float d[R][C];

    /** Đặt tất cả phần tử = 0 */
    inline void zero() {
        memset(d, 0, sizeof(d));
    }

    /** Đặt ma trận đơn vị (chỉ cho ma trận vuông) */
    inline void identity() {
        zero();
        constexpr int n = (R < C) ? R : C;
        for (int i = 0; i < n; i++) {
            d[i][i] = 1.0f;
        }
    }

    /** Truy cập phần tử */
    inline float& operator()(int r, int c) { return d[r][c]; }
    inline float  operator()(int r, int c) const { return d[r][c]; }
};

/****************************************************************************
 * Type Aliases
 ****************************************************************************/

using Mat9   = Mat<9, 9>;
using Mat3   = Mat<3, 3>;
using Mat3x9 = Mat<3, 9>;
using Mat9x3 = Mat<9, 3>;
using Vec9   = Mat<9, 1>;
using Vec3   = Mat<3, 1>;

/****************************************************************************
 * Matrix Operations
 ****************************************************************************/

/**
 * Nhân ma trận: C = A × B
 * A: R×K, B: K×C → Result: R×C
 * Compiler tự suy luận kích thước từ tham số
 */
template<int R, int K, int C>
inline Mat<R, C> mat_mul(const Mat<R, K>& A, const Mat<K, C>& B) {
    Mat<R, C> result;
    for (int i = 0; i < R; i++) {
        for (int j = 0; j < C; j++) {
            float sum = 0.0f;
            for (int k = 0; k < K; k++) {
                sum += A.d[i][k] * B.d[k][j];
            }
            result.d[i][j] = sum;
        }
    }
    return result;
}

/**
 * Chuyển vị: A^T
 * A: R×C → Result: C×R
 */
template<int R, int C>
inline Mat<C, R> mat_transpose(const Mat<R, C>& A) {
    Mat<C, R> result;
    for (int i = 0; i < R; i++) {
        for (int j = 0; j < C; j++) {
            result.d[j][i] = A.d[i][j];
        }
    }
    return result;
}

/**
 * Cộng ma trận: C = A + B
 */
template<int R, int C>
inline Mat<R, C> mat_add(const Mat<R, C>& A, const Mat<R, C>& B) {
    Mat<R, C> result;
    for (int i = 0; i < R; i++) {
        for (int j = 0; j < C; j++) {
            result.d[i][j] = A.d[i][j] + B.d[i][j];
        }
    }
    return result;
}

/**
 * Trừ ma trận: C = A - B
 */
template<int R, int C>
inline Mat<R, C> mat_sub(const Mat<R, C>& A, const Mat<R, C>& B) {
    Mat<R, C> result;
    for (int i = 0; i < R; i++) {
        for (int j = 0; j < C; j++) {
            result.d[i][j] = A.d[i][j] - B.d[i][j];
        }
    }
    return result;
}

/**
 * Nhân scalar: B = A × s
 */
template<int R, int C>
inline Mat<R, C> mat_scale(const Mat<R, C>& A, float s) {
    Mat<R, C> result;
    for (int i = 0; i < R; i++) {
        for (int j = 0; j < C; j++) {
            result.d[i][j] = A.d[i][j] * s;
        }
    }
    return result;
}

/**
 * Nghịch đảo ma trận 3×3 (công thức giải tích Cramer)
 * Dùng cho S^-1 trong bước Update ESKF
 */
inline Mat3 mat3_inverse(const Mat3& A) {
    Mat3 inv;

    float det = A.d[0][0] * (A.d[1][1] * A.d[2][2] - A.d[1][2] * A.d[2][1])
              - A.d[0][1] * (A.d[1][0] * A.d[2][2] - A.d[1][2] * A.d[2][0])
              + A.d[0][2] * (A.d[1][0] * A.d[2][1] - A.d[1][1] * A.d[2][0]);

    if (fabsf(det) < 1e-10f) {
        inv.identity();
        return inv;
    }

    float inv_det = 1.0f / det;

    inv.d[0][0] =  (A.d[1][1] * A.d[2][2] - A.d[1][2] * A.d[2][1]) * inv_det;
    inv.d[0][1] = -(A.d[0][1] * A.d[2][2] - A.d[0][2] * A.d[2][1]) * inv_det;
    inv.d[0][2] =  (A.d[0][1] * A.d[1][2] - A.d[0][2] * A.d[1][1]) * inv_det;

    inv.d[1][0] = -(A.d[1][0] * A.d[2][2] - A.d[1][2] * A.d[2][0]) * inv_det;
    inv.d[1][1] =  (A.d[0][0] * A.d[2][2] - A.d[0][2] * A.d[2][0]) * inv_det;
    inv.d[1][2] = -(A.d[0][0] * A.d[1][2] - A.d[0][2] * A.d[1][0]) * inv_det;

    inv.d[2][0] =  (A.d[1][0] * A.d[2][1] - A.d[1][1] * A.d[2][0]) * inv_det;
    inv.d[2][1] = -(A.d[0][0] * A.d[2][1] - A.d[0][1] * A.d[2][0]) * inv_det;
    inv.d[2][2] =  (A.d[0][0] * A.d[1][1] - A.d[0][1] * A.d[1][0]) * inv_det;

    return inv;
}

/**
 * Đối xứng hóa ma trận: A = 0.5 × (A + A^T)
 * Đảm bảo ma trận hiệp phương sai luôn đối xứng
 */
template<int N>
inline void mat_symmetrize(Mat<N, N>& A) {
    for (int i = 0; i < N; i++) {
        for (int j = i + 1; j < N; j++) {
            float avg = 0.5f * (A.d[i][j] + A.d[j][i]);
            A.d[i][j] = avg;
            A.d[j][i] = avg;
        }
    }
}

/**
 * Ma trận phản đối xứng (skew-symmetric) từ vector 3D
 *
 *         |  0   -vz   vy |
 * [v]× =  |  vz   0   -vx |
 *         | -vy   vx   0  |
 */
inline Mat3 skew(const float v[3]) {
    Mat3 S;
    S.d[0][0] =  0.0f;   S.d[0][1] = -v[2];   S.d[0][2] =  v[1];
    S.d[1][0] =  v[2];   S.d[1][1] =  0.0f;   S.d[1][2] = -v[0];
    S.d[2][0] = -v[1];   S.d[2][1] =  v[0];   S.d[2][2] =  0.0f;
    return S;
}

} // namespace eskf

#endif // UAV_LIB_ESKF_MATRIX_HPP
