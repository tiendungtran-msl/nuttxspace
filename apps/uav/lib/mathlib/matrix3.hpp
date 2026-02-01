/****************************************************************************
 * apps/uav/lib/mathlib/matrix3.hpp
 *
 * 3x3 Matrix Class cho rotation và transformation
 *
 * MỤC ĐÍCH:
 * - Biểu diễn rotation matrix (DCM)
 * - Matrix operations: multiply, transpose, inverse
 * - Transform vectors
 *
 * LƯU Ý:
 * - Row-major storage: M[row][col]
 * - m[0], m[1], m[2] là row 0
 *
 ****************************************************************************/

#ifndef UAV_LIB_MATHLIB_MATRIX3_HPP
#define UAV_LIB_MATHLIB_MATRIX3_HPP

#include <cmath>
#include <cstring>

namespace mathlib {

/****************************************************************************
 * Matrix3 Class
 ****************************************************************************/

class Matrix3 {
public:
    float m[3][3];  // Row-major: m[row][col]

    //=========================================================================
    // Constructors
    //=========================================================================

    // Default: identity matrix
    Matrix3() {
        m[0][0] = 1; m[0][1] = 0; m[0][2] = 0;
        m[1][0] = 0; m[1][1] = 1; m[1][2] = 0;
        m[2][0] = 0; m[2][1] = 0; m[2][2] = 1;
    }

    // From flat array (row-major)
    explicit Matrix3(const float* arr) {
        memcpy(m, arr, 9 * sizeof(float));
    }

    // From components
    Matrix3(float m00, float m01, float m02,
            float m10, float m11, float m12,
            float m20, float m21, float m22) {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
    }

    //=========================================================================
    // Factory Methods
    //=========================================================================

    /**
     * Tạo rotation matrix từ Euler angles (ZYX convention)
     */
    static Matrix3 from_euler(float roll, float pitch, float yaw) {
        float cr = cosf(roll);
        float sr = sinf(roll);
        float cp = cosf(pitch);
        float sp = sinf(pitch);
        float cy = cosf(yaw);
        float sy = sinf(yaw);

        Matrix3 R;
        R.m[0][0] = cp * cy;
        R.m[0][1] = cp * sy;
        R.m[0][2] = -sp;

        R.m[1][0] = sr * sp * cy - cr * sy;
        R.m[1][1] = sr * sp * sy + cr * cy;
        R.m[1][2] = sr * cp;

        R.m[2][0] = cr * sp * cy + sr * sy;
        R.m[2][1] = cr * sp * sy - sr * cy;
        R.m[2][2] = cr * cp;

        return R;
    }

    /**
     * Tạo skew-symmetric matrix từ vector
     * [v]_x = | 0  -vz  vy |
     *         | vz  0  -vx |
     *         |-vy vx   0  |
     */
    static Matrix3 skew(const float v[3]) {
        Matrix3 S;
        S.m[0][0] = 0;      S.m[0][1] = -v[2];  S.m[0][2] = v[1];
        S.m[1][0] = v[2];   S.m[1][1] = 0;      S.m[1][2] = -v[0];
        S.m[2][0] = -v[1];  S.m[2][1] = v[0];   S.m[2][2] = 0;
        return S;
    }

    //=========================================================================
    // Operations
    //=========================================================================

    /**
     * Transpose
     */
    Matrix3 transpose() const {
        Matrix3 T;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                T.m[i][j] = m[j][i];
            }
        }
        return T;
    }

    /**
     * Determinant
     */
    float det() const {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
             - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
             + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    /**
     * Inverse (cho rotation matrix: inverse = transpose)
     */
    Matrix3 inverse() const {
        float d = det();
        if (fabsf(d) < 1e-10f) {
            return Matrix3();  // Return identity if singular
        }

        float inv_d = 1.0f / d;
        Matrix3 I;

        I.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_d;
        I.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_d;
        I.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_d;

        I.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_d;
        I.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_d;
        I.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_d;

        I.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_d;
        I.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_d;
        I.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_d;

        return I;
    }

    /**
     * Matrix multiplication
     */
    Matrix3 operator*(const Matrix3& other) const {
        Matrix3 R;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R.m[i][j] = m[i][0] * other.m[0][j]
                          + m[i][1] * other.m[1][j]
                          + m[i][2] * other.m[2][j];
            }
        }
        return R;
    }

    /**
     * Matrix-vector multiplication: v_out = M * v_in
     */
    void multiply_vector(const float v_in[3], float v_out[3]) const {
        v_out[0] = m[0][0] * v_in[0] + m[0][1] * v_in[1] + m[0][2] * v_in[2];
        v_out[1] = m[1][0] * v_in[0] + m[1][1] * v_in[1] + m[1][2] * v_in[2];
        v_out[2] = m[2][0] * v_in[0] + m[2][1] * v_in[1] + m[2][2] * v_in[2];
    }

    /**
     * Addition
     */
    Matrix3 operator+(const Matrix3& other) const {
        Matrix3 R;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R.m[i][j] = m[i][j] + other.m[i][j];
            }
        }
        return R;
    }

    /**
     * Scalar multiplication
     */
    Matrix3 operator*(float s) const {
        Matrix3 R;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R.m[i][j] = m[i][j] * s;
            }
        }
        return R;
    }

    //=========================================================================
    // Accessors
    //=========================================================================

    /**
     * Element access
     */
    float& operator()(int row, int col) {
        return m[row][col];
    }

    float operator()(int row, int col) const {
        return m[row][col];
    }

    /**
     * Get row vector
     */
    void get_row(int row, float v[3]) const {
        v[0] = m[row][0];
        v[1] = m[row][1];
        v[2] = m[row][2];
    }

    /**
     * Get column vector
     */
    void get_col(int col, float v[3]) const {
        v[0] = m[0][col];
        v[1] = m[1][col];
        v[2] = m[2][col];
    }

    /**
     * Copy to flat array (row-major)
     */
    void to_array(float arr[9]) const {
        memcpy(arr, m, 9 * sizeof(float));
    }
};

} // namespace mathlib

#endif // UAV_LIB_MATHLIB_MATRIX3_HPP
