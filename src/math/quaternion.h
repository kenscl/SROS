#ifndef __QUATERNION
#define __QUATERNION
#include "matrix.h"
#include "vector.h"
#include <cstddef>
#include <cstdint>

class Quaternion {
    public:
        double q, i, j, k;
    public:
        Quaternion();
        Quaternion(double w, double i, double j, double k);
        Quaternion(const Quaternion & other);
        Quaternion(const Vector & vec);
        Quaternion(const Matrix & mat);

        Quaternion operator*(const Quaternion& other) const;
        Quaternion operator+(const Quaternion& other) const;
        Quaternion operator*(const double& nbr) const;

        Quaternion operator*(const Matrix& other) const;

        Quaternion& operator=(const Quaternion& other);
        Quaternion conjugate() const;
        Vector to_rpy() const;
        Vector to_vector() const;
        Matrix to_rotation_matrix() const;

        double norm() const;
        Quaternion normalize() const;

        void print() const;

};

#endif
