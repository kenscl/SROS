#include "quaternion.h"
#include <cmath>
#include "vector.h"
#include "../communication/usart.h"

Quaternion::Quaternion() : q(1), i(0), j(0), k(0) {}
Quaternion::Quaternion(double w, double i, double j, double k) : q(w), i(i), j(j), k(k) {}
Quaternion::Quaternion(const Quaternion &other) {
    this->q = other.q;
    this->i = other.i;
    this->j = other.j;
    this->k = other.k;
}
Quaternion::Quaternion(const Vector &vec) {
    this->q = vec[0];
    this->i = vec[1];
    this->j = vec[2];
    this->k = vec[3];
}

Quaternion::Quaternion(const Matrix &mat) {
    if (mat.m != 3 || mat.n != 3) {
        OS_WARN("rotation matrix must be 3x3");
    }
    Quaternion ret = Quaternion(1, 0, 0, 0);

    double trace = mat[0][0] + mat[1][1] + mat[2][2];

    if (trace > 0) {
        double s = 0.5 / sqrt(trace + 1);
        this->q = 0.25 / s;
        this->i = (mat[2][1] - mat[1][2]) * s;
        this->j = (mat[0][2] - mat[2][0]) * s;
        this->k = (mat[1][0] - mat[0][1]) * s;
    } else {
        if (mat[0][0] > mat[1][1] && mat[0][0] > mat[2][2]) {
        double s = 2 * sqrt(1 + mat[0][0] - mat[1][1] - mat[2][2]);
        this->q = (mat[2][1] - mat[1][2]) / s;
        this->i = 0.25 * s;
        this->j = (mat[0][1] + mat[1][0]) / s;
        this->k = (mat[0][2] + mat[2][0]) / s;
      } else if (mat[1][1] > mat[2][2]) {
        double s = 2 * sqrt(1 + mat[1][1] - mat[0][0] - mat[2][2]);
        this->q = (mat[0][2] - mat[2][0]) / s;
        this->i = (mat[0][1] + mat[1][0]) / s;
        this->j = 0.25 * s;
        this->k = (mat[1][2] + mat[2][1]) / s;
      } else {
        double s = 2 * sqrt(1 + mat[2][2] - mat[0][0] - mat[1][1]);
        this->q = (mat[1][0] - mat[0][1]) / s;
        this->i = (mat[0][2] + mat[2][0]) / s;
        this->j = (mat[1][2] + mat[2][1]) / s;
        this->k = 0.25 * s;
      }
    }
}


Quaternion Quaternion::operator*(const Quaternion &other) const {
     return Quaternion(
        q * other.q - i * other.i - j * other.j - k * other.k,  
        q * other.i + i * other.q + j * other.k - k * other.j,  
        q * other.j - i * other.k + j * other.q + k * other.i,  
        q * other.k + i * other.j - j * other.i + k * other.q   
    );
}

Quaternion Quaternion::operator+(const Quaternion &other) const {
    return Quaternion(
        q + other.q,
        i + other.i,
        j + other.j,
        k + other.k
    );
}

Quaternion Quaternion::operator*(const double &nbr) const {
    return Quaternion(q * nbr, i * nbr, j * nbr, k * nbr);
}

Quaternion Quaternion::operator*(const Matrix &other) const {
    if (other.m != 4) {
        OS_WARN("Q err matrix");
        return Quaternion();
    }
    Vector v = Vector(4);
    v[0] = this->q;
    v[1] = this->i;
    v[2] = this->j;
    v[3] = this->k;
    v = other * v;
    return Quaternion(v[0],v[1],v[2],v[3]);
   
}

Quaternion & Quaternion::operator=(const Quaternion &other) {
    this->q = other.q;
    this->i = other.i;
    this->j = other.j;
    this->k = other.k;
    return *this;
}

Quaternion Quaternion::conjugate() const {
     return Quaternion(q, -i, -j, -k);
}

Vector Quaternion::to_rpy() const {
    Vector result(3);
    result[0] = atan2(2 * (q * i + j * k), 1 - 2 * (i * i + j * j));
    result[1] = asin(2 * (q * j - k * i));
    result[2] = atan2(2 * (q * k + i * j), 1 - 2 * (j * j + k * k));
    return result;
 
}

Vector Quaternion::to_vector() const {
    Vector ret(4);
    ret[0] = q;
    ret[1] = i;
    ret[2] = j;
    ret[3] = k;
}

Matrix Quaternion::to_rotation_matrix() const {
    Matrix result(3, 3);
    result[0][0] = 1 - 2 * (j * j + k * k);
    result[0][1] = 2 * (i * j - k * q);
    result[0][2] = 2 * (i * k + j * q);
    result[1][0] = 2 * (i * j + k * q);
    result[1][1] = 1 - 2 * (i * i + k * k);
    result[1][2] = 2 * (j * k - i * q);
    result[2][0] = 2 * (i * k - j * q);
    result[2][1] = 2 * (j * k + i * q);
    result[2][2] = 1 - 2 * (i * i + j * j);
    return result;
}

double Quaternion::norm() const {
    return this->to_vector().norm();
}

Quaternion Quaternion::normalize() const {
    double norm = this->norm();
    return Quaternion(q / norm, i / norm, j / norm, k / norm);
}

void Quaternion::print() const {
    os_printf("Quaternion: \n");
    os_printf("q: %f, i: %f, j: %f, k: %f \n", q, i, j, k);
}
