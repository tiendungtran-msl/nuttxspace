/****************************************************************************
 * apps/examples/uav_states_v1/lib/ekf2/ekf_matrix.hpp
 *
 * Ma trận đơn giản cho EKF2 - Tối ưu cho embedded
 *
 * MỤC ĐÍCH:
 * - Cung cấp các phép toán ma trận cơ bản cho Kalman filter.
 * - Tối ưu bộ nhớ và tốc độ cho MCU.
 * - Không dùng dynamic allocation.
 *
 * CHÚ Ý:
 * - Ma trận covariance P là SquareMatrix<16>.
 * - Có thể dùng Cholesky decomposition để tối ưu sau này.
 *
 ****************************************************************************/

#pragma once

#include <cstring>
#include <cmath>

namespace ekf2
{

/****************************************************************************
 * Matrix - Ma trận MxN
 ****************************************************************************/

template<int M, int N>
class Matrix
{
public:
    float data[M][N];

    Matrix()
    {
        zero();
    }

    void zero()
    {
        memset(data, 0, sizeof(data));
    }

    // Thiết lập đường chéo (chỉ cho ma trận vuông)
    void setDiag(float val)
    {
        zero();
        int min_dim = (M < N) ? M : N;
        for (int i = 0; i < min_dim; i++) {
            data[i][i] = val;
        }
    }

    // Truy cập phần tử
    float& operator()(int row, int col) { return data[row][col]; }
    float operator()(int row, int col) const { return data[row][col]; }

    // Lấy kích thước
    static constexpr int rows() { return M; }
    static constexpr int cols() { return N; }

    // Cộng ma trận
    Matrix<M, N> operator+(const Matrix<M, N>& other) const
    {
        Matrix<M, N> result;
        for (int i = 0; i < M; i++) {
            for (int j = 0; j < N; j++) {
                result.data[i][j] = data[i][j] + other.data[i][j];
            }
        }
        return result;
    }

    // Trừ ma trận
    Matrix<M, N> operator-(const Matrix<M, N>& other) const
    {
        Matrix<M, N> result;
        for (int i = 0; i < M; i++) {
            for (int j = 0; j < N; j++) {
                result.data[i][j] = data[i][j] - other.data[i][j];
            }
        }
        return result;
    }

    // Nhân với scalar
    Matrix<M, N> operator*(float s) const
    {
        Matrix<M, N> result;
        for (int i = 0; i < M; i++) {
            for (int j = 0; j < N; j++) {
                result.data[i][j] = data[i][j] * s;
            }
        }
        return result;
    }

    // Nhân ma trận: (M x N) * (N x P) = (M x P)
    template<int P>
    Matrix<M, P> operator*(const Matrix<N, P>& other) const
    {
        Matrix<M, P> result;
        for (int i = 0; i < M; i++) {
            for (int j = 0; j < P; j++) {
                float sum = 0;
                for (int k = 0; k < N; k++) {
                    sum += data[i][k] * other.data[k][j];
                }
                result.data[i][j] = sum;
            }
        }
        return result;
    }

    // Transpose
    Matrix<N, M> T() const
    {
        Matrix<N, M> result;
        for (int i = 0; i < M; i++) {
            for (int j = 0; j < N; j++) {
                result.data[j][i] = data[i][j];
            }
        }
        return result;
    }

    // Inplace operations
    Matrix<M, N>& operator+=(const Matrix<M, N>& other)
    {
        for (int i = 0; i < M; i++) {
            for (int j = 0; j < N; j++) {
                data[i][j] += other.data[i][j];
            }
        }
        return *this;
    }

    Matrix<M, N>& operator-=(const Matrix<M, N>& other)
    {
        for (int i = 0; i < M; i++) {
            for (int j = 0; j < N; j++) {
                data[i][j] -= other.data[i][j];
            }
        }
        return *this;
    }
};

/****************************************************************************
 * SquareMatrix - Ma trận vuông NxN
 ****************************************************************************/

template<int N>
class SquareMatrix : public Matrix<N, N>
{
public:
    using Matrix<N, N>::data;
    using Matrix<N, N>::zero;

    SquareMatrix() : Matrix<N, N>() {}

    // Ma trận đơn vị
    void setIdentity()
    {
        zero();
        for (int i = 0; i < N; i++) {
            data[i][i] = 1.0f;
        }
    }

    // Lấy đường chéo
    void getDiag(float diag[N]) const
    {
        for (int i = 0; i < N; i++) {
            diag[i] = data[i][i];
        }
    }

    // Set variance cho một nhóm states
    void setBlockDiag(int start_idx, int size, float variance)
    {
        for (int i = 0; i < size; i++) {
            data[start_idx + i][start_idx + i] = variance;
        }
    }

    // Reset covariance của một block
    void resetBlock(int start_idx, int size, float variance)
    {
        // Zero out rows and columns
        for (int i = start_idx; i < start_idx + size; i++) {
            for (int j = 0; j < N; j++) {
                data[i][j] = 0;
                data[j][i] = 0;
            }
        }
        // Set diagonal
        setBlockDiag(start_idx, size, variance);
    }

    // Giới hạn variance
    void constrainVariance(int idx, float min_val, float max_val)
    {
        if (data[idx][idx] < min_val) {
            data[idx][idx] = min_val;
        } else if (data[idx][idx] > max_val) {
            data[idx][idx] = max_val;
        }
    }

    // Đảm bảo ma trận đối xứng (tránh sai số tích lũy)
    void makeSymmetric()
    {
        for (int i = 0; i < N; i++) {
            for (int j = i + 1; j < N; j++) {
                float avg = 0.5f * (data[i][j] + data[j][i]);
                data[i][j] = avg;
                data[j][i] = avg;
            }
        }
    }

    // P = P + Q (thêm process noise)
    void addDiag(const float q[N])
    {
        for (int i = 0; i < N; i++) {
            data[i][i] += q[i];
        }
    }
};

/****************************************************************************
 * Vector - Vector cột N phần tử
 ****************************************************************************/

template<int N>
class Vector
{
public:
    float data[N];

    Vector()
    {
        zero();
    }

    void zero()
    {
        memset(data, 0, sizeof(data));
    }

    float& operator()(int i) { return data[i]; }
    float operator()(int i) const { return data[i]; }

    float& operator[](int i) { return data[i]; }
    float operator[](int i) const { return data[i]; }

    static constexpr int size() { return N; }

    // Norm
    float norm() const
    {
        float sum = 0;
        for (int i = 0; i < N; i++) {
            sum += data[i] * data[i];
        }
        return sqrtf(sum);
    }

    // Cộng vector
    Vector<N> operator+(const Vector<N>& other) const
    {
        Vector<N> result;
        for (int i = 0; i < N; i++) {
            result.data[i] = data[i] + other.data[i];
        }
        return result;
    }

    // Trừ vector
    Vector<N> operator-(const Vector<N>& other) const
    {
        Vector<N> result;
        for (int i = 0; i < N; i++) {
            result.data[i] = data[i] - other.data[i];
        }
        return result;
    }

    // Nhân với scalar
    Vector<N> operator*(float s) const
    {
        Vector<N> result;
        for (int i = 0; i < N; i++) {
            result.data[i] = data[i] * s;
        }
        return result;
    }

    // Inplace
    Vector<N>& operator+=(const Vector<N>& other)
    {
        for (int i = 0; i < N; i++) {
            data[i] += other.data[i];
        }
        return *this;
    }

    Vector<N>& operator-=(const Vector<N>& other)
    {
        for (int i = 0; i < N; i++) {
            data[i] -= other.data[i];
        }
        return *this;
    }
};

/****************************************************************************
 * Outer product: v * v^T = Matrix
 ****************************************************************************/

template<int N>
SquareMatrix<N> outer_product(const Vector<N>& v)
{
    SquareMatrix<N> result;
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            result.data[i][j] = v.data[i] * v.data[j];
        }
    }
    return result;
}

/****************************************************************************
 * Matrix-Vector multiplication: M * v
 ****************************************************************************/

template<int M, int N>
Vector<M> operator*(const Matrix<M, N>& mat, const Vector<N>& vec)
{
    Vector<M> result;
    for (int i = 0; i < M; i++) {
        float sum = 0;
        for (int j = 0; j < N; j++) {
            sum += mat.data[i][j] * vec.data[j];
        }
        result.data[i] = sum;
    }
    return result;
}

/****************************************************************************
 * Scalar * Matrix
 ****************************************************************************/

template<int M, int N>
Matrix<M, N> operator*(float s, const Matrix<M, N>& mat)
{
    return mat * s;
}

/****************************************************************************
 * Scalar * Vector
 ****************************************************************************/

template<int N>
Vector<N> operator*(float s, const Vector<N>& vec)
{
    return vec * s;
}

} // namespace ekf2
