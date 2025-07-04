#ifndef __QUATERNION
#define __QUATERNION
#include "matrix.h"
#include "vector.h"
#include <cstddef>
#include <cstdint>

class Quaternion {
    public:
        float q, i, j, k;
    public:
        Quaternion() : q(1), i(0), j(0), k(0) {}
        Quaternion(float w, float i, float j, float k) : q(w), i(i), j(j), k(k) {}

        Quaternion(float roll, float pitch, float yaw) {
          float cr = cos(roll * 0.5);
          float sr = sin(roll * 0.5);
          float cp = cos(pitch * 0.5);
          float sp = sin(pitch * 0.5);
          float cy = cos(yaw * 0.5);
          float sy = sin(yaw * 0.5);

          this->q = cr * cp * cy + sr * sp * sy;
          this->i = sr * cp * cy - cr * sp * sy;
          this->j = cr * sp * cy + sr * cp * sy;
          this->k = cr * cp * sy - sr * sp * cy;
        }

        Quaternion(Vec3 rpy) {
	  float roll = rpy[0];
	  float pitch = rpy[1];
	  float yaw = rpy[2];
          float cr = cos(roll * 0.5);
          float sr = sin(roll * 0.5);
          float cp = cos(pitch * 0.5);
          float sp = sin(pitch * 0.5);
          float cy = cos(yaw * 0.5);
          float sy = sin(yaw * 0.5);

          this->q = cr * cp * cy + sr * sp * sy;
          this->i = sr * cp * cy - cr * sp * sy;
          this->j = cr * sp * cy + sr * cp * sy;
          this->k = cr * cp * sy - sr * sp * cy;
        }

        Quaternion(const Quaternion &other) {
          this->q = other.q;
          this->i = other.i;
          this->j = other.j;
          this->k = other.k;
        }

        Quaternion(const Vec4 &vec) {
          this->q = vec[0];
          this->i = vec[1];
          this->j = vec[2];
          this->k = vec[3];
        }
        Quaternion(const Mat3 & mat) {
            Quaternion ret = Quaternion(1, 0, 0, 0);

            float trace = mat[0][0] + mat[1][1] + mat[2][2];

            if (trace > 0) {
                float s = 0.5 / sqrt(trace + 1);
                this->q = 0.25 / s;
                this->i = (mat[2][1] - mat[1][2]) * s;
                this->j = (mat[0][2] - mat[2][0]) * s;
                this->k = (mat[1][0] - mat[0][1]) * s;
            } else {
                if (mat[0][0] > mat[1][1] && mat[0][0] > mat[2][2]) {
                float s = 2 * sqrt(1 + mat[0][0] - mat[1][1] - mat[2][2]);
                this->q = (mat[2][1] - mat[1][2]) / s;
                this->i = 0.25 * s;
                this->j = (mat[0][1] + mat[1][0]) / s;
                this->k = (mat[0][2] + mat[2][0]) / s;
            } else if (mat[1][1] > mat[2][2]) {
                float s = 2 * sqrt(1 + mat[1][1] - mat[0][0] - mat[2][2]);
                this->q = (mat[0][2] - mat[2][0]) / s;
                this->i = (mat[0][1] + mat[1][0]) / s;
                this->j = 0.25 * s;
                this->k = (mat[1][2] + mat[2][1]) / s;
            } else {
                float s = 2 * sqrt(1 + mat[2][2] - mat[0][0] - mat[1][1]);
                this->q = (mat[1][0] - mat[0][1]) / s;
                this->i = (mat[0][2] + mat[2][0]) / s;
                this->j = (mat[1][2] + mat[2][1]) / s;
                this->k = 0.25 * s;
            }
            }
        }

        Quaternion operator*(const Quaternion& other) const {
            return Quaternion(
                q * other.q - i * other.i - j * other.j - k * other.k,  
                q * other.i + i * other.q + j * other.k - k * other.j,  
                q * other.j - i * other.k + j * other.q + k * other.i,  
                q * other.k + i * other.j - j * other.i + k * other.q   
            );
        }

        Quaternion operator+(const Quaternion& other) const {
            return Quaternion(
                q + other.q,
                i + other.i,
                j + other.j,
                k + other.k
            );
        }

        Quaternion operator*(const float& nbr) const{
            return Quaternion(q * nbr, i * nbr, j * nbr, k * nbr);
        }

        Quaternion operator*(const Mat4& other) const {
            Vec4 v;
            v[0] = this->q;
            v[1] = this->i;
            v[2] = this->j;
            v[3] = this->k;
            v = other * v;
            return Quaternion(v[0],v[1],v[2],v[3]);

        }

        Quaternion& operator=(const Quaternion& other) {
            this->q = other.q;
            this->i = other.i;
            this->j = other.j;
            this->k = other.k;
            return *this;
        }

        Quaternion conjugate() const {
            return Quaternion(q, -i, -j, -k);
        }

        Vec3 to_rpy() const {
            Vec3 result;
            result[0] = atan2(2 * (q * i + j * k), 1 - 2 * (i * i + j * j));
            result[1] = asin(2 * (q * j - k * i));
            result[2] = atan2(2 * (q * k + i * j), 1 - 2 * (j * j + k * k));
            return result;

        }

        Vec4 to_vector() const {
            Vec4 ret;
            ret[0] = q;
            ret[1] = i;
            ret[2] = j;
            ret[3] = k;
            return ret;
        }

        Mat3 to_rotation_matrix() const {
            Mat3 result;
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

        float norm() const {
            return this->to_vector().norm();
        }

        Quaternion normalize() const {
            float norm = this->norm();
            if (norm == 0) norm = 1;
            return Quaternion(q / norm, i / norm, j / norm, k / norm);
        }

        void print() const {
            os_putstr("Quaternion: \n", 13);
            os_putstr("q: ", 3);
            os_putf(q);
            os_putstr("i: ", 3);
            os_putf(i);
            os_putstr("j: ", 3);
            os_putf(j);
            os_putstr("k: ", 3);
            os_putf(k);
            os_putstr("\n", 1);
        }

        void print_bare() const {
            os_printf("%f, %f, %f, %f \n", q, i, j, k);
        }
};

#endif
