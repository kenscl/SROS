#ifndef __VECTOR
#define __VECTOR
#include <cstddef>
#include <cstdint>
#include "../krnl/mem.h"
class Matrix;
class Vector {
    protected:
      size_t size;
      double *r;

    public:
      Vector();
      Vector(size_t size);
      Vector(size_t size, double *r);
      Vector(const Vector &other);
      ~Vector();

      Vector operator*(double d) const;
      double operator*(Vector other) const;
      Vector operator/(double d) const;
      Vector operator-(const Vector other) const;
      Vector operator+(const Vector other) const;
      Vector &operator=(const Vector &other);
      uint8_t operator==(const Vector &other) const;

      const double& operator[](size_t index) const;
      double& operator[](size_t index);


      double norm() const;
      Vector normalize();
      Vector sub_vector(size_t bottom, size_t top);


      void print();
      void print_bare();

    //memory allocation
    // do not use these they cause bugs for some reason
    //void *operator new(size_t size) { return math_alloc(size); }
    //void *operator new[](size_t size) { return math_alloc(size); }

    //void operator delete(void *ptr) { return math_free(ptr); }
    //void operator delete[](void *ptr) { return math_free(ptr); }
    friend Matrix;
};

#endif __VECTOR
