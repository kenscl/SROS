#ifndef __EKF
#define __EKF
#include "../math/matrix.h"
#include "../math/quaternion.h"
#include "../sensors/LSM9DS1.h"
#include <stdint.h>
#define M_PI 3.14159265358979323846264338327950288419716939937510

typedef struct EKF {
    // data
    Quat *attitude;
    Vec *bias; // 3
    // EKF components
    Vec *x; // 7
    Vec *h, *z, *y; // 4
    Mat *P, *F, *Q; //7 x 7
    Mat *H; // 4 x 7
    Mat *K;// 7 x 4
    Mat *R; // 4 x 4
    Mat *Rot; // 3 x 3
    Mat *Rot_inv; // 3 x 3
    // helper data
    Quat *q;

    Mat *temp_mat1; // 7 x 7
    Mat *temp_mat2; // 7 x 7
    Mat *F_trans; // 7 x 7

    Mat *S; // 4 x 4
    Mat *S_inv; // 4 x 4
    Mat *H_trans; // 7 x 4
    Mat *tmp1; // 7 x 4
    Mat *tmp2; //4 x 4

    Vec *tmp; // 7

    Mat *i7; // 7 x 7
    Mat *tmp3; // 7 x 7
    Mat *tmp4; // 7 x 7
} EKF;

int EKF_alloc(struct EKF **ekf);
void EKF_init_incremental(EKF *ekf, Vec *gyro, Vec *acc, Vec *mag);
void EKF_update_acc(EKF *ekf, Vec *acc);
void EKF_update_mag(EKF *ekf, Vec *mag, Vec *acc);
void EKF_predict(EKF *ekf,Vec *gyro, float dt);
void EKF_update(EKF *ekf);

volatile void attitude_thread();

#endif
