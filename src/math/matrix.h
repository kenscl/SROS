#ifndef __MATRIX
#define __MATRIX
#include "vector.h"
#include <cstddef>
#include <cstdint>

class Quaternion;

class Matrix {
    protected:
      size_t m; // ^
      size_t n; // ->
      Vector *r;

    public:
        Matrix();
        Matrix(size_t m, size_t n);
        Matrix(size_t m, size_t n, Vector * data);
        Matrix(const Matrix & other);
        ~Matrix();
        const Vector &operator[](size_t m) const;
        Vector &operator[](size_t m);

        uint8_t operator==(const Matrix &other) const; 
        Matrix operator+(const Matrix &other) const;
        Matrix operator-(const Matrix &other) const;
        Matrix operator*(const Matrix &other) const;
        Matrix &operator=(const Matrix &other);

        Matrix operator*(double scalar) const;
        Matrix operator/(double scalar) const;
        Vector operator*(const Vector & v) const;

        Matrix submatrix(size_t row_to_remove, size_t col_to_remove);
        double det();
        Matrix adjugate();
        Matrix transpose();
        Matrix inverse();
        Matrix identity();

        Matrix diag(double d);
        Matrix diag(Vector v);

        void print();
        void *operator new(size_t size) { return math_alloc(size); }
        void operator delete(void *ptr) { return math_free(ptr); }
    friend Quaternion;
};

#endif
