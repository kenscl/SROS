#include <cstddef>
#include <cstdint>
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


      void print();
};

