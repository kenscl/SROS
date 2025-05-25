#ifndef __COMPLEMENTARY_FILTER
#define __COMPLEMENTARY_FILTER
#include "../math/matrix.h"
#include "../math/quaternion.h"
#include "../communication/LSM9DS1.h"
#include <cstdint>
#define M_PI 3.14159265358979323846264338327950288419716939937510 

class Complementary_Filter{
private:
  float roll;
  float pitch;
  float yaw;
  Vec3 prediction_am;
  float beta;

public:
  Quaternion attitude;
  void init(Vec3 *gyro, Vec3 *acc, Vec3 *mag);
  void update_acc(Vec3 acc);
  void update_mag(Vec3 mag);
  void predict(Vec3 gyro, float dt);
};

volatile void attitude_thread_complementary_filter();

#endif
