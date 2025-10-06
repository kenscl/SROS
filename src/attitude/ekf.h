#ifndef __EKF
#define __EKF
#include "../math/matrix.h"
#include "../math/quaternion.h"
//#include "../communication/LSM9DS1.h"
#include <stdint.h>
#define M_PI 3.14159265358979323846264338327950288419716939937510

typedef struct EKF {
    Quat attitude;
    Vec bias; // 3
    Vec x; // 10
    Vec y, z; // 4
    Mat P, F, Q; // 10 x 10
    Mat H; // 4 x 10
    Mat K;// 10 x 4
    Mat R; // 4 x 4
    Mat Rot; // 3 x 3
    Mat Rot_inv; // 3 x 3
} EKF;

void EKF_init(Vec *gyro, Vec *acc, Vec *mag);
void EKF_update_acc(Vec acc);
void EKF_update_mag(Vec mag, Vec acc);
void EKF_predict(Vec gyro, float dt);
void EKF_update();

volatile void attitude_thread();

#endif
