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
    Vec *x; // 10
    Vec *y, *z; // 4
    Mat *P, *F, *Q; // 10 x 10
    Mat *H; // 4 x 10
    Mat *K;// 10 x 4
    Mat *R; // 4 x 4
    Mat *Rot; // 3 x 3
    Mat *Rot_inv; // 3 x 3
    // helper data
    Quat *q;
    Quat *w;
    // temp data
    Quat *q_temp;

    Mat *temp_mat1; // 10 x 10
    Mat *temp_mat2; // 10 x 10
    Mat *F_trans; // 10 x 10

    Vec *rev_g; // 3
    Vec *z_acc; // 3
    Vec *rpy; // 3

    Vec *v; // 4

    Mat *S; // 4 x 4
    Mat *S_inv; // 4 x 4
    Mat *H_trans; // 10 x 4
    Mat *tmp1; // 10 x 4
    Mat *tmp2; //4 x 4

    Vec *tmp; // 10

    Mat *i10; // 10 x 10
    Mat *tmp3; // 10 x 10
    Mat *tmp4; // 10 x 10

    Vec *m; // 3
    Vec *mn; // 3
} EKF;

int EKF_alloc(struct EKF **ekf);
void EKF_init(EKF *ekf, Vec **gyro, Vec **acc, Vec **mag);
void EKF_update_acc(EKF *ekf, Vec *acc);
void EKF_update_mag(EKF *ekf, Vec *mag, Vec *acc);
void EKF_predict(EKF *ekf,Vec *gyro, float dt);
void EKF_update(EKF *ekf);

volatile void attitude_thread();

#endif
