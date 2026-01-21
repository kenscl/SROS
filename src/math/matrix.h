#ifndef __MATRIX
#define __MATRIX
#include "vector.h"
#include <stddef.h>
#include <stdint.h>

class Quat;

template <size_t m, size_t n> class Mat {
  protected:
    float r[m][n]{};

  public:
    Mat() {}
    Mat(const Mat &other) {
        for (size_t i = 0; i < m; ++i) {
            for (size_t j = 0; j < n; ++j) {
                this->r[i][j] = other.r[i][j];
            }
        }
    }
    ~Mat() {}
    const float *operator[](size_t index) const {
        if (m < index) {
            OS_WARN("Out of Bounds in Mat acces, returning 0 element!");
            return r[0];
        }
        return r[index];
    }

    float *operator[](size_t index) {
        if (m < index) {
            OS_WARN("Out of Bounds in Mat acces, returning 0 element!");
            return r[0];
        }
        return r[index];
    }

    uint8_t operator==(const Mat &other) const {
        for (size_t i = 0; i < m; ++i) {
            for (size_t j = 0; j < n; ++j) {
                if (r[i][j] != other.r[i][j])
                    return false;
            }
        }
        return true;
    }

    template <size_t p> Mat<m, p> operator*(const Mat<n, p> &other) const {
        Mat<m, p> result;
        for (size_t i = 0; i < m; ++i) {
            for (size_t j = 0; j < p; ++j) {
                result[i][j] = 0;
                for (size_t k = 0; k < n; ++k) {
                    result[i][j] += r[i][k] * other[k][j];
                }
            }
        }
        return result;
    }

    Mat operator+(const Mat &other) const {
        Mat result;
        for (size_t i = 0; i < m; ++i) {
            for (size_t j = 0; j < n; ++j) {
                result.r[i][j] = r[i][j] + other.r[i][j];
            }
        }
        return result;
    }

    Mat operator-(const Mat &other) const {
        Mat result;
        for (size_t i = 0; i < m; ++i) {
            for (size_t j = 0; j < n; ++j) {
                result.r[i][j] = r[i][j] - other.r[i][j];
            }
        }
        return result;
    }

    Mat operator*(float scalar) const {
        Mat result;
        for (size_t i = 0; i < m; ++i) {
            for (size_t j = 0; j < n; ++j) {
                result.r[i][j] = r[i][j] * scalar;
            }
        }
        return result;
    }

    Mat operator/(float scalar) const {
        if (scalar == 0) {
            OS_WARN("M err div");
            return *this;
        }
        Mat result;
        for (size_t i = 0; i < m; ++i) {
            for (size_t j = 0; j < n; ++j) {
                result.r[i][j] = r[i][j] / scalar;
            }
        }
        return result;
    }

    Mat &operator=(const Mat &other) {
        if (this != &other) {
            for (size_t i = 0; i < m; ++i) {
                for (size_t j = 0; j < n; ++j) {
                    r[i][j] = other.r[i][j];
                }
            }
        }
        return *this;
    }

    Vec<m> operator*(const Vec<n> &v) const {
        Vec<m> result;
        for (size_t i = 0; i < m; ++i) {
            float sum = 0.0;
            for (size_t j = 0; j < n; ++j) {
                sum += r[i][j] * v[j];
            }
            result[i] = sum;
        }
        return result;
    }

    Mat<m - 1, n - 1> minor(size_t row, size_t col) const {
        Mat<m - 1, n - 1> result;
        size_t r_idx = 0;
        for (size_t i = 0; i < m; ++i) {
            if (i == row) continue;
            size_t c_idx = 0;
            for (size_t j = 0; j < n; ++j) {
                if (j == col) continue;
                result[r_idx][c_idx] = r[i][j];
                ++c_idx;
            }
            ++r_idx;
        }
	return result;
    }

    float det() __attribute__((optimize("O0"))){
        if constexpr (m == 1) {
            return r[0][0];
        } else if constexpr (m == 2) {
            return r[0][0] * r[1][1] - r[0][1] * r[1][0];
        } else if constexpr (m == 3) {
            return r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) -
                   r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
                   r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
        } else {
            // General recursive formula
            volatile float det = 0.0;
            for (size_t j = 0; j < n; ++j) {
                float m_det = minor(0, j).det();
                det += ((j % 2 == 0) ? 1 : -1) * r[0][j] * m_det;
            }
            return det;
        }
    }

    // cofactor for the adjugate since submatrix causes issues.
    float coaf(size_t i, size_t j) {
        Mat<m - 1, n - 1> minor;
	int minor_row = 0, minor_col = 0;
	float sign = ((i + j) % 2 == 0) ? 1 : -1;

        for (int row = 0; row < m; ++row) {
            if (row == i)
                continue;
            minor_col = 0;
            for (int col = 0; col < n; ++col) {
                if (col == j)
                    continue;
                minor[minor_row][minor_col] = r[row][col];
                minor_col++;
            }
            minor_row++;
        }
        float res = sign * minor.det();
        return res;
    }

    Mat adjugate() {
        if (m != n) {
            OS_WARN("M err adjugate!");
            return *this;
        }
        Mat adj;
        if (n == 1) {
            adj[0][0] = 1;
            return adj;
        }

        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                float cofactor = this->coaf(i, j);
                adj[j][i] = cofactor;
            }
        }
        return adj;
    }

    Mat<n, m> transpose() {
        Mat<n, m> result;
        for (size_t i = 0; i < m; ++i) {
            for (size_t j = 0; j < n; ++j) {
                result[j][i] = r[i][j];
            }
        }
        return result;
    }

    Mat inverse() {
        float det = this->det();
        if (n != m || det == 0) {
            os_printf("M err inverse! \n");
            while (1) {
            }
            return *this;
        }
        Mat adj = this->adjugate();
        return adj / det;
    }

    Mat identity() {
        Mat ret;
        for (int i = 0; i < n; ++i) {
            ret[i][i] = 1;
        }
        return ret;
    }

    Mat diag(float d) {
        if (n != m)
            OS_WARN("M diag!");
        Mat ret(*this);
        for (int i = 0; i < n; ++i) {
            ret[i][i] = d;
        }
        return ret;
    }

    Mat diag(Vec<m> v) {
        if (n != m)
            OS_WARN("M diag!");
        Mat ret(*this);
        for (int i = 0; i < n; ++i) {
            ret[i][i] = v[i];
        }
        return ret;
    }

    void print() {
        os_putstr("Mat: \n");
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; j++) {
                if (r[i][j] != r[i][j])
                    os_putstr("NaN \n");
                else
                    os_putf(r[i][j]);
                os_putstr(" ");
            }
            os_putstr("\n");
        }
    }

    friend Quat;
};

typedef Mat<4, 4> Mat4;
typedef Mat<3, 3> Mat3;
typedef Mat<2, 2> Mat2;

#endif
