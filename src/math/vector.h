#ifndef __VECTOR
#define __VECTOR
#include <cmath>
#include <cstddef>
#include <cstdint>
#include "../krnl/mem.h"
#include "../communication/usart.h"
//class Matrix;
template <size_t size>
class Vec {
    protected:
    float r[size]{};

    public:
        Vec () {}
        Vec(float *arr) {
            for (size_t i = 0; i < size; i++) {
                r[i] = arr[i];
            }
        }

        Vec(const Vec &other) {
            for (size_t i = 0; i < size; i++) {
                r[i] = other.r[i];
            }
        }

        ~Vec() {}

        Vec operator*(float d) const {
            Vec result;
            for (size_t i = 0; i < size; i++) {
                result.r[i] = r[i] * d;
            }
            return result;
        }

        float operator*(Vec other) const {
            float dot_product = 0.0;
            for (size_t i = 0; i < size; i++) {
                dot_product += r[i] * other.r[i];
            }
            return dot_product;
        }

        Vec mult(Vec other) const {
            Vec res;
            for (size_t i = 0; i < size; i++) {
                res[i] = this->r[i] * other.r[i];
            }
            return res;
        }

        Vec operator/(float d) const {
            if (d == 0) {
                OS_WARN("Vector div by 0!");
                return *this;
            }
            Vec result;
            for (size_t i = 0; i < size; i++) {
                result.r[i] = r[i] / d;
            }
            return result;
        }

        Vec operator-(const Vec other) const {
            Vec result;
            for (size_t i = 0; i < size; i++) {
                result.r[i] = r[i] - other.r[i];
            }
            return result;
        }

        Vec operator+(const Vec other) const {
            Vec result;
            for (size_t i = 0; i < size; i++) {
                result.r[i] = r[i] + other.r[i];
            }
            return result;
        }

        Vec &operator=(const Vec &other) {
            if (this != &other) {
                for (size_t i = 0; i < size; i++) {
                    r[i] = other.r[i];
                }
            }
            return *this;
        }

        uint8_t operator==(const Vec &other) const {
            for (size_t i = 0; i < size; i++) {
                if (r[i] != other.r[i]) return 0;
            }
            return 1;
        }

        const float& operator[](size_t index) const {
            if (index >= size) OS_WARN("V err Index out of bounds");
            return r[index];
        }

        float& operator[](size_t index) {
            if (index >= size) OS_WARN("V err Index out of bounds");
            return r[index];
        }

        float norm() const {
            float sum = 0;
            for (int i = 0; i < size; ++i) {
                sum += this->r[i] * this->r[i];
            }
            return sqrt(sum);
        }

        Vec normalize() {
                float norm = this->norm();
                if (norm == 0)
                return Vec(size);
                Vec ret(*this / norm);
                return ret;
        }

        void print() {
            os_putstr("Vector: \n", 9);
            for (int i = 0; i < size; ++i) {
              if (r[i] != r[i])
                  os_putstr("NaN \n",5);
              else {
                os_putf(this->r[i]);
                os_putstr("\n",1);
              }
            }
        }

        void print_bare() {
            for (int i = 0; i < size - 1; ++i) {
                if (r[i] != r[i])
                    os_putstr("NaN \n",5);
                else {
                    os_putf(this->r[i]);
                    os_putstr(", ",2);
                }
            }
            os_putf(this->r[size-1]);
            os_putstr("\n",1);
        }
};
typedef Vec<4> Vec4;
typedef Vec<3> Vec3;
typedef Vec<2> Vec2;

#endif __VECTOR
