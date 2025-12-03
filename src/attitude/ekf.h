#ifndef __EKF
#define __EKF
#include "../math/matrix.h"
#include "../math/quaternion.h"
#include "../sensors/LSM9DS1.h"
#include <stdint.h>
#define M_PI 3.14159265358979323846264338327950288419716939937510
#define BIAS_INSTABILITY 1e-11
#define CALIB_COUNT      50

typedef struct EKF {
    // data
    Quat attitude;
    Vec3 bias; // 3
    // EKF components
    Vec<4> x; // 7
    Vec<6> h, z, y; // 6
    Mat<4, 4> P, F, Q; //7 x 7
    Mat<6, 4> H; // 4 x 7
    Mat<4, 6> K;// 4 x 4
    Mat<6, 6> R; // 6 x 6
    // helper data
    Quat q;

    Mat<4, 4> temp_mat1; // 4 x 4
    Mat<4, 4> temp_mat2; // 4 x 4
    Mat<4, 4> F_trans; // 4 x 4
    Mat<4, 3> W; // 3 x 4
    Mat<3, 4> W_trans; // 4 x 3

    Mat<4, 4> S; // 4 x 4
    Mat<4, 4> S_inv; // 4 x 4
    Mat<4, 6> H_trans; // 4 x 4


    Mat<4, 4> i4; // 4 x 4
    Mat<3, 3> rot; // 3 x 3
    Mat<3, 3> rot_inv; // 3 x 3
    Vec3 acc_refrence; // 3
    Vec3 mag_refrence; // 3
    Vec3 vtmp; // 3

    float gyro_variance;
} EKF;

void EKF_init_incremental(EKF *ekf, Vec3 *gyro, Vec3 *acc, Vec3 *mag);
void EKF_init_final(EKF *ekf);
void EKF_update_acc(EKF *ekf, Vec3 *acc);
void EKF_update_mag(EKF *ekf, Vec3 *mag, Vec3 *acc);
void EKF_predict(EKF *ekf,Vec3 *gyro, float dt);
void EKF_update(EKF *ekf);

volatile void attitude_thread();

#endif
