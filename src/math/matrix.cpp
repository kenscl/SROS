#include "matrix.h"
#include "../krnl/mem.h"
#include "../communication/usart.h"
#include "vector.h"
#include <cmath>

Matrix::Matrix() {
    this->m = 0;
    this->n = 0;
    this->r = nullptr;
}

Matrix::Matrix(size_t m, size_t n) {
    this->m = m;
    this->n = n;
    this->r = (Vector *) math_alloc(m * sizeof(Vector));
    for (size_t i = 0; i < m; ++i) {
        this->r[i] = Vector(n);
    }
}

Matrix::Matrix(size_t m, size_t n, Vector *data) : m(m), n(n) {
    this->r = (Vector *) math_alloc(m * sizeof(Vector));
    for (size_t i = 0; i < m; ++i) {
        this->r[i] = data[i];
    }
}

Matrix::Matrix(const Matrix &other) : m(other.m), n(other.n) {
    this->r = (Vector *) math_alloc(m * sizeof(Vector));
    for (size_t i = 0; i < m; ++i) {
        this->r[i] = other.r[i];
    }
    
}

Matrix::~Matrix() {
    os_free(r);
}


const Vector &Matrix::operator[](size_t m) const {
    if (this->m < m) {
        OS_WARN("Out of Bounds in Matrix acces, returning 0 element!");
        return r[0];
    }
    return r[m];
}

Vector &Matrix::operator[](size_t m) {
    if (this->m < m) {
        OS_WARN("Out of Bounds in Matrix acces, returning 0 element!");
        return r[0];
    }
    return r[m];
}

uint8_t Matrix::operator==(const Matrix &other) const {
    if (m != other.m || n != other.n)
      return false;
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            if (r[i][j] != other.r[i][j])
                return false;
        }
    }
    return true;
}

Matrix Matrix::operator+(const Matrix &other) const {
    if (m != other.m || n != other.n) {
        OS_WARN("MERR addition");
        return *this;
    }
    Matrix result(*this);
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            result[i][j] += other[i][j];
        }
    }
    return result;
}

Matrix Matrix::operator-(const Matrix &other) const {
    if (m != other.m || n != other.n) {
        OS_WARN("MERR subtraction");
        return *this;
    }
    Matrix result(*this);
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            result[i][j] -= other[i][j];
        }
    }
    return result;
}

Matrix Matrix::operator*(const Matrix &other) const {
    if (n != other.m) {
        OS_WARN("MERR matrix multiplication");
    }
    Matrix result(m, other.n);
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < other.n; ++j) {
            result[i][j] = 0;
            for (size_t k = 0; k < n; ++k) {
                result[i][j] += r[i][k] * other.r[k][j];
            }
        }
    }
    return result;
}


Matrix &Matrix::operator=(const Matrix &other) {
    this->m= other.m;
    this->n= other.n;

    if (this->r) { 
        for (int i = 0; i < other.m; ++i) {
            math_free(&this->r[i]);
        }
        math_free(this->r);
    }
    this->r = (Vector *) math_alloc(other.m* sizeof(Vector));
    //os_printf("alloc in copy at: %d %d \n", this->r, other.r);
    for (int i = 0; i < this->m; ++i) {
        this->r[i] = other.r[i];
    }
    return *this;
}

Matrix Matrix::operator*(double scalar) const {
    Matrix result(*this);
    for (size_t i = 0; i < m; ++i) {
        result[i] = result[i] * scalar;
    }
    return result;
}

Matrix Matrix::operator/(double scalar) const {
    Matrix result(*this);
    for (size_t i = 0; i < m; ++i) {
        result[i] = result[i] / scalar;
    }
    return result;
}

Vector Matrix::operator*(const Vector &v) const {
    if (v.size != this->n) {
        OS_WARN("M err Matrix * Vector");
        return v;
    }
    Vector result(m);
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            result[i] += this->r[i][j] * v[j];
        }
    }
    return result;

}

Matrix Matrix::submatrix(size_t row_to_remove, size_t col_to_remove) {
    Matrix result(n - 1, n - 1);
    size_t new_i = 0;

    for (size_t i = 0; i < n; ++i) {
        if (i == row_to_remove)
            continue;

        size_t new_j = 0;
        for (size_t j = 0; j < n; ++j) {
            if (j == col_to_remove)
                continue;

            result[new_i][new_j] = r[i][j];
            new_j++;
        }
        new_i++;
    }

    return result;
}

double Matrix::det() {
    if (m != n) {
        OS_WARN("M Err det of non square matrix!");
    }
    if (n == 1) return this->r[0][0];
    Matrix temp = *this;;
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
          Vector t = temp[i];
          temp[i] = temp[max_row];
          temp[max_row] = t;
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


Matrix Matrix::adjugate() {
    if (m != n) {
        OS_WARN("M err adjugate!");
    }
    Matrix adj(n,n);
    if (n == 1) { 
        adj[0][0] = 1;
        return adj;
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            Matrix minor_matrix = submatrix(i, j);
            double determinant = minor_matrix.det(); 

            adj[j][i] = ((i + j) % 2 == 0 ? 1 : -1) * determinant;
        }
    }
    return adj;
}

Matrix Matrix::transpose() {
    Matrix result(n, m);
       for (size_t i = 0; i < m; ++i) {
           for (size_t j = 0; j < n; ++j) {
               result[j][i] = r[i][j]; 
           }
       }
    return result;
}

Matrix Matrix::inverse() {
    double det = this->det();
    if (n != m || det == 0) {
        OS_WARN("M err inverse!");
    }
    Matrix adj = this->adjugate();
    return adj / det; 
}

Matrix Matrix::identity() {
    Matrix ret(n,n);
    for (int i = 0; i < n; ++i) {
        ret[i][i] = 1;
    }
    return ret;
}

Matrix Matrix::diag(double d) {
    if (n != m)
        OS_WARN("M diag!");
    Matrix ret(*this);
    for (int i = 0; i < n; ++i) {
        ret[i][i] = d;
    }
    return ret;
}

Matrix Matrix::diag(Vector v) {
    if (n != m)
        OS_WARN("M diag!");
    Matrix ret(*this);
    for (int i = 0; i < n; ++i) {
        ret[i][i] = v[i];
    }
    return ret;
}

void Matrix::print() {
    os_printf("Matrix: \n");
    for (int i = 0; i < this->m; ++i) {
        this->r[i].print_bare();
    }
}
