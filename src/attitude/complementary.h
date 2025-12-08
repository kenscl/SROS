#ifndef __COMPLEMENTARY
#define __COMPLEMENTARY
#include "../math/matrix.h"
#include "../math/quaternion.h"
#include "../sensors/LSM9DS1.h"
#include <stdint.h>
#define M_PI 3.14159265358979323846264338327950288419716939937510

void complementary_update(Vec3 *acc, Vec3* mag);
void complementary_predict(Vec3 *gyro, float dt);

volatile void complementary_thread();

#endif
