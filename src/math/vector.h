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
    double r[size]{};

    public:
        Vec () {}
        Vec(double *arr) {
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

        Vec operator*(double d) const {
            Vec result;
            for (size_t i = 0; i < size; i++) {
                result.r[i] = r[i] * d;
            }
            return result;
        }

        double operator*(Vec other) const {
            double dot_product = 0.0;
            for (size_t i = 0; i < size; i++) {
                dot_product += r[i] * other.r[i];
            }
            return dot_product;
        }

        Vec operator/(double d) const {
            if (d == 0) {
                OS_WARN("Vector div by 0!");
                return this;
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

        const double& operator[](size_t index) const {
            if (index >= size) OS_WARN("V err Index out of bounds");
            return r[index];
        }

        double& operator[](size_t index) {
            if (index >= size) OS_WARN("V err Index out of bounds");
            return r[index];
        }

        double norm() const {
            double sum = 0;
            for (int i = 0; i < size; ++i) {
                sum += this->r[i] * this->r[i];
            }
            return sqrt(sum);
        }

        Vec normalize() {
                double norm = this->norm();
                if (norm == 0)
                return Vec(size);
                Vec ret(*this / norm);
                return ret;
        }

        void print() {
            os_printf("Vector: \n");
            for (int i = 0; i < size; ++i) {
                os_printf("%f \n", this->r[i]);
            }
        }

        void print_bare() {
            for (int i = 0; i < size; ++i) {
                os_printf("%f ", this->r[i]);
            }
            os_printf("\n");
        }
    //memory allocation
    // do not use these they cause bugs for some reason
    //void *operator new(size_t size) { return math_alloc(size); }
    //void *operator new[](size_t size) { return math_alloc(size); }

    //void operator delete(void *ptr) { return math_free(ptr); }
    //void operator delete[](void *ptr) { return math_free(ptr); }
    //friend Matrix;
};
typedef Vec<4> Vec4D;
typedef Vec<3> Vec3D;
typedef Vec<2> Vec2D;

#endif __VECTOR
