#ifndef __MATRIX
#define __MATRIX
#include "vector.h"
#include <cstddef>
#include <cstdint>

class Quaternion;

template <size_t m, size_t n>
class Mat {
    protected:
    double r[m][n]{};

    public:
        Mat() {}
        Mat(const Mat & other) {
            for (size_t i = 0; i < m; ++i) {
                for (size_t j = 0; j < n; ++j) {
                    this->r[i][j] = other.r[i][j];
                }
            }
        }
        ~Mat() {}
        const double *operator[](size_t index) const {
            if (m < index) {
                OS_WARN("Out of Bounds in Mat acces, returning 0 element!");
                return r[0];
            }
            return r[index];
        }

        double *operator[](size_t index) {
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

        template <size_t p>
        Mat<m, p> operator*(const Mat<n, p> &other) const {
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

        Mat operator*(double scalar) const {
            Mat result;
            for (size_t i = 0; i < m; ++i) {
                for (size_t j = 0; j < n; ++j) {
                    result.r[i][j] = r[i][j] * scalar;
                }
            }
            return result;
        }

        Mat operator/(double scalar) const {
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

        Mat& operator=(const Mat &other) {
            if (this != &other) {
                for (size_t i = 0; i < m; ++i) {
                    for (size_t j = 0; j < n; ++j) {
                        r[i][j] = other.r[i][j];
                    }
                }
            }
            return *this;
        }

        Vec<m> operator*(Vec<n> &v) const {
            Vec<m> result;
            for (size_t i = 0; i < m; ++i) {
                double sum = 0.0;
                for (size_t j = 0; j < n; ++j) {
                    sum += r[i][j] * v[j];
                }
                result[i] = sum;
            }
            return result;
        }


        double det() {
            if (m != n) {
                OS_WARN("Error: Determinant of non-square matrix!");
                return 0;
            }
            if (n == 1) return this->r[0][0];
            Mat<m, n> temp = *this; 
            double det = 1;

            for (int i = 0; i < n; ++i) {
                double max_elem = temp[i][i];
                int max_row = i;
                for (int k = i + 1; k < n; ++k) {
                    if (fabs(temp[k][i]) > fabs(max_elem)) {
                        max_elem = temp[k][i];
                        max_row = k;
                    }
                }

                if (max_row != i) {
                    for (int j = 0; j < n; ++j) {
                        double temp_val = temp[i][j];
                        temp[i][j] = temp[max_row][j];
                        temp[max_row][j] = temp_val;
                    }
                    det *= -1;
                }
                if (temp[i][i] == 0) {
                    return 0;
                }

                for (int k = i + 1; k < n; ++k) {
                    double factor = temp[k][i] / temp[i][i];
                    for (int j = i; j < n; ++j) {
                        temp[k][j] -= factor * temp[i][j];
                    }
                }

                det *= temp[i][i];
            }

            return det;
        }

        // cofactor for the adjugate since submatrix causes issues.
        double coaf(size_t i, size_t j) {
            Mat<m - 1, n - 1> minor;
            int minor_row = 0, minor_col = 0;
            double sign = ((i + j) % 2 == 0) ? 1 : -1;

            for (int row = 0; row < m; ++row) {
                if (row == i) continue;
                minor_col = 0;
                for (int col = 0; col < n; ++col) {
                    if (col == j) continue; 
                    minor[minor_row][minor_col] = r[row][col];
                    minor_col++;
                }
                minor_row++;
            }
            return sign * minor.det();
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
                            double cofactor = this->coaf(i, j);
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
            double det = this->det();
            if (n != m || det == 0) {
                OS_WARN("M err inverse!");
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

        Mat diag(double d) {
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
          os_printf("Mat: \n");
          for (int i = 0; i < m; ++i) {
              for (int j = 0; j < n; j++) {
                if (r[i][j] != r[i][j])
                  os_printf("NaN \n");
                else
                  os_printf("%f ", r[i][j]);
              }
              os_printf("\n");
          }
        }

    friend Quaternion;
};

typedef Mat<4,4> Mat4;
typedef Mat<3,3> Mat3;
typedef Mat<2,2> Mat2;

#endif
