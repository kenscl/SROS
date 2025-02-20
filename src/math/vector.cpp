#include "vector.h"
#include "../krnl/mem.h"
#include "../communication/usart.h"
#include <cmath>

Vector::Vector() {
    this->size = 0;
    this->r = nullptr;
}

Vector::Vector(size_t size) {
    this->size = size;
    this->r = (double *) math_alloc(size * sizeof(double));
    if (this->r == (double* ) nullptr) OS_WARN("Out of Math Memory! Do not allocate more memory!"); // Obviously this only works in development and not when a system is deployed.
}

Vector::Vector(size_t size, double *r) {
    this->size = size;
    this->r = r; 
}
Vector::Vector(const Vector &other) {
    this->size = other.size;
    this->r = (double *) math_alloc(size * sizeof(double));
    for (int i = 0; i < this->size; ++i) {
        this->r[i] = other.r[i];
    }
}

Vector::~Vector() {
    math_free(this->r);
}

Vector Vector::operator*(double d) const {
    Vector result (this->size);
    for (int i = 0; i < this->size; ++i) {
        result.r[i] = this->r[i] * d;
    }
    return result;
}

double Vector::operator*(Vector other) const {
    if (this->size != other.size) OS_WARN("Vector sizes are not equal in dot product!");
    double total = 0; 
    for (int i = 0; i < this->size; ++i) {
        total += this->r[i] * other.r[i];
    }
    return total;
}

Vector Vector::operator/(double d) const {
    Vector result (this->size);
    for (int i = 0; i < this->size; ++i) {
        result.r[i] = this->r[i] / d;
    }
    return result;
}

Vector Vector::operator-(const Vector other) const {
    if (this->size != other.size) OS_WARN("Vector sizes are not equal in subtraction!");
    Vector result (this->size);
    for (int i = 0; i < this->size; ++i) {
        result.r[i] = this->r[i] - other.r[i];
    }
    return result;
}

Vector Vector::operator+(const Vector other) const {
    if (this->size != other.size) OS_WARN("Vector sizes are not equal in addition!");
    Vector result (this->size);
    for (int i = 0; i < this->size; ++i) {
        result.r[i] = this->r[i] + other.r[i];
    }
    return result;
}


Vector & Vector::operator=(const Vector &other) {
    this->size = other.size;
    if (this->r) math_free(this->r);
    this->r = (double *) math_alloc(size * sizeof(double));
    for (int i = 0; i < this->size; ++i) {
        this->r[i] = other.r[i];
    }
    return *this;
}

uint8_t Vector::operator==(const Vector &other) const {
    if (this->size != other.size) return false;
    for (int i = 0; i < this->size; ++i) {
        if (this->r[i] != other.r[i]) return false;
    }
    return true;
}

double &Vector::operator[](size_t index) {
    if (this->size < index) {
        OS_WARN("Out of Bounds in Vector acces, returning 0 element!");
        return r[0];
    }
    return r[index];
}

const double& Vector::operator[](size_t index) const {
    if (this->size < index) {
        OS_WARN("Out of Bounds in Vector acces, returning 0 element!");
        return r[0];
    }
    return r[index];
}

double Vector::norm() const {
    double sum = 0;
    for (int i = 0; i < this->size; ++i) {
        sum += this->r[i] * this->r[i];
    }
    return sqrt(sum);
}

Vector Vector::normalize() {
    double norm = this->norm();
    if (norm == 0) return Vector(this->size);
    Vector ret(*this / norm);
    return ret;
}

void Vector::print() {
    os_printf("Vector: \n");
    for (int i = 0; i < this->size; ++i) {
        os_printf("%f \n", this->r[i]);
    }
}
