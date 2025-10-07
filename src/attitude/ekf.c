#include "ekf.h"
#include <math.h>
#include <stdint.h>


void EKF_init(EKF *ekf, Vec **gyro, Vec **acc, Vec **mag) {
    int num_init = 100;
    Vec *mean_gyro = vec_alloc(3);
    Vec *mean_acc = vec_alloc(3);
    Vec *mean_mag = vec_alloc(3);

    float init_div = 1 / num_init;
    for (int i = 0; i < num_init; ++i) {
        vec_add(mean_gyro, gyro[i], mean_gyro);
        vec_scalar_mult(mean_gyro, init_div);
        vec_sub(mean_acc, acc[i], mean_acc);
        vec_scalar_mult(mean_acc, init_div);
        vec_add(mean_mag, mag[i], mean_mag);
        vec_scalar_mult(mean_mag, init_div);
    }

    float roll_init = atan2(mean_acc->r[1], mean_acc->r[2]);
    float pitch_init = atan2(mean_acc->r[0], sqrt(mean_acc->r[1]*mean_acc->r[1]+mean_acc->r[2]*mean_acc->r[2]));

    float cp = cos(pitch_init);
    float sp = sin(pitch_init);
    float cr = cos(roll_init);
    float sr = sin(roll_init);

    ekf->Rot->r[0] = cp;
    ekf->Rot->r[1] = sp * sr;
    ekf->Rot->r[2] = sp * cr;

    ekf->Rot->r[3] = 0;
    ekf->Rot->r[4] = cr;
    ekf->Rot->r[5] = - sr;

    ekf->Rot->r[6] = -sp;
    ekf->Rot->r[7] = cp * sr;
    ekf->Rot->r[8] = cp * cr;

    Vec *mr = vec_alloc(3);
    mat_vec_mult(ekf->Rot, mean_mag, mr);
    float yaw_init = atan2(-mr->r[1], mr->r[0]);
    Quat *q_init = quat_alloc();(roll_init, pitch_init, yaw_init);
    quat_from_rpy(q_init, roll_init, pitch_init, yaw_init);

    ekf->x->r[0] = q_init->q;
    ekf->x->r[1] = q_init->i;
    ekf->x->r[2] = q_init->j;
    ekf->x->r[3] = q_init->k;
    ekf->x->r[7] = mean_gyro->r[0];
    ekf->x->r[8] = mean_gyro->r[1];
    ekf->x->r[9] = mean_gyro->r[2];

    mat_identity(ekf->P);
    mat_scalar_mult(ekf->P, 10.f);

    float v_bias = 1.0e-11;

    Vec *gyro_sum = vec_alloc(3);
    Vec *acc_sum = vec_alloc(3);
    float mag_sum;

    Vec *temp = vec_alloc(3);
    Vec *mw = vec_alloc(3);
    for (int i = 0; i < num_init; ++i) {
        //gyro_sum = gyro_sum + (gyro[i] - mean_gyro).mult(gyro[i] - mean_gyro);
        vec_sub(gyro[i], mean_gyro, gyro[i]);
        vec_mult(gyro[i], gyro[i], temp);
        vec_add(gyro_sum, temp, gyro_sum);

        //acc_sum = acc_sum + (acc[i] + mean_acc).mult(acc[i] + mean_acc);
        vec_add(acc[i], mean_acc, acc[i]);
        vec_mult(acc[i], acc[i], temp);
        vec_add(acc_sum, temp, acc_sum);

        //Vec3 mw = ekf->Rot * mag[i];
        mat_vec_mult(ekf->Rot, mag[i], mw);
        float mag_yaw = atan2(-mw->r[1], mw->r[0]);
        mag_sum = mag_sum + (mag_yaw - yaw_init) * (mag_yaw - yaw_init);
    }
    vec_free(temp);
    vec_free(mw);

    float s_gyro = (sqrt(gyro_sum->r[0]/(num_init-1)) + sqrt(gyro_sum->r[1]/(num_init-1)) + sqrt(gyro_sum->r[2]/(num_init-1))) / 3;
    float s_acc = (sqrt(acc_sum->r[0]/(num_init-1)) + sqrt(acc_sum->r[1]/(num_init-1)) + sqrt(acc_sum->r[2]/(num_init-1))) / 3;
    float s_yaw = sqrt(mag_sum/(num_init-1));


    ekf->R->r[0] = 2.12425429e-06;
    ekf->R->r[5] = 2.12425429e-06;
    ekf->R->r[10] = 2.12425429e-06;
    ekf->R->r[15] = 7.98243297e-05;


    Mat *Fu = mat_alloc(10, 6);
    Mat *Fu_trans = mat_alloc(6, 10);
    Fu->r[4 * 6 +0] = 1;
    Fu->r[5 * 6 +1] = 1;
    Fu->r[6 * 6 +2] = 1;
    Fu->r[7 * 6 +3] = 1;
    Fu->r[8 * 6 +4] = 1;
    Fu->r[9 * 6 +5] = 1;


    float q0 = ekf->x->r[0];
    float q1 = ekf->x->r[1];
    float q2 = ekf->x->r[2];
    float q3 = ekf->x->r[3];

    ekf->Rot->r[0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    ekf->Rot->r[1] = 2 * (q1 * q2 - q0 * q3);
    ekf->Rot->r[2] = 2 * (q0 * q2 + q1 * q3);

    ekf->Rot->r[3] = 2 * (q1 * q2 + q0 * q3);
    ekf->Rot->r[4] = (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3);
    ekf->Rot->r[5] = 2 * (q2 * q3 - q0 * q1);

    ekf->Rot->r[6] = 2 * (q1 * q3 - q0 * q2);
    ekf->Rot->r[7] = 2 * (q0 * q1 + q2 * q3);
    ekf->Rot->r[8] = (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    mat_transpose(ekf->Rot, ekf->Rot_inv);

    Mat *U = mat_alloc(6, 6);
    U->r[0 * 6 + 0] = 3.47930986e-04;
    U->r[1 * 6 + 1] = 3.47930986e-04;
    U->r[2 * 6 + 2] = 3.47930986e-04;
    U->r[3 * 6 + 3] = v_bias;
    U->r[4 * 6 + 4] = v_bias;
    U->r[5 * 6 + 5] = v_bias;
    //ekf->Q = Fu * U * Fu.transpose();
    Mat *temp_mat = mat_alloc(6, 10);
    mat_mult(U, Fu_trans, temp_mat);
    mat_mult(Fu, temp_mat, ekf->Q);

    mat_free(temp_mat);
    mat_free(U);
    mat_free(Fu);
    mat_free(Fu_trans);
}

void EKF_update_acc(EKF *ekf, Vec *acc) {
    ekf->y->r[0] = acc->r[0];
    ekf->y->r[1] = acc->r[1];
    ekf->y->r[2] = acc->r[2];
}

void EKF_update_mag(EKF *ekf, Vec *mag, Vec *acc) {
    Vec *m = vec_alloc(3);
    Vec *mn = vec_alloc(3);
    mat_vec_mult(ekf->Rot, mag, m);
    m->r[2] = 0;

    //m =  ekf->Rot_inv * m;
    mat_vec_mult(ekf->Rot_inv, m, mn);

    float yaw = atan2(- mn->r[1], mn->r[0]);
    ekf->y->r[3] = yaw;
    vec_free(m);
    vec_free(mn);
}

void EKF_predict(EKF *ekf,Vec *gyro, float dt) {
    float q0 = ekf->x->r[0];
    float q1 = ekf->x->r[1];
    float q2 = ekf->x->r[2];
    float q3 = ekf->x->r[3];
    float wx = ekf->x->r[4];
    float wy = ekf->x->r[5];
    float wz = ekf->x->r[6];
    float xgx = ekf->x->r[7];
    float xgy = ekf->x->r[8];
    float xgz = ekf->x->r[9];

    Quat *q = quat_alloc();
    (ekf->x[0], ekf->x[1], ekf->x[2], ekf->x[3]);
    quat_from_vec4(ekf->x, q);

    Quat *w = quat_alloc();
    w->q = 0;
    w->i = wx;
    w->j = wy;
    w->k = wz;

    //q = q + q * w * 0.5 * dt;
    Quat *temp = quat_alloc();
    quat_mult(q, w, temp);
    quat_scalar_mult(temp, 0.5f * dt);
    quat_add(q, temp, q);
    quat_normalize(q);
    quat_free(temp);

    ekf->x->r[0] = q->q;
    ekf->x->r[1] = q->i;
    ekf->x->r[2] = q->j;
    ekf->x->r[3] = q->k;
    ekf->x->r[4] = gyro->r[0] - xgx;
    ekf->x->r[5] = gyro->r[1] - xgy;
    ekf->x->r[6] = gyro->r[2] - xgz;
    ekf->x->r[7] = xgx;
    ekf->x->r[8] = xgy;
    ekf->x->r[9] = xgz;

    ekf->F->r[0 * 10 + 0] = 1;
    ekf->F->r[0 * 10 + 1] = -0.5 * dt * wx;
    ekf->F->r[0 * 10 + 2] = -0.5 * dt * wy;
    ekf->F->r[0 * 10 + 3] = -0.5 * dt * wz;
    ekf->F->r[0 * 10 + 4] = -0.5 * dt * q1;
    ekf->F->r[0 * 10 + 5] = -0.5 * dt * q2;
    ekf->F->r[0 * 10 + 6] = -0.5 * dt * q3;
    ekf->F->r[0 * 10 + 7] = 0;
    ekf->F->r[0 * 10 + 8] = 0;
    ekf->F->r[0 * 10 + 9] = 0;

    ekf->F->r[1 * 10 + 0] = 0.5 * dt * wx;
    ekf->F->r[1 * 10 + 1] = 1;
    ekf->F->r[1 * 10 + 2] = 0.5 * dt * wz;
    ekf->F->r[1 * 10 + 3] = -0.5 * dt * wy;
    ekf->F->r[1 * 10 + 4] = 0.5 * dt * q0;
    ekf->F->r[1 * 10 + 5] = -0.5 * dt * q3;
    ekf->F->r[1 * 10 + 6] = 0.5 * dt * q2;
    ekf->F->r[1 * 10 + 7] = 0;
    ekf->F->r[1 * 10 + 8] = 0;
    ekf->F->r[1 * 10 + 9] = 0;

    ekf->F->r[2 * 10 + 0] = 0.5 * dt * wy;
    ekf->F->r[2 * 10 + 1] = -0.5 * dt * wz;
    ekf->F->r[2 * 10 + 2] = 1;
    ekf->F->r[2 * 10 + 3] = 0.5 * dt * wx;
    ekf->F->r[2 * 10 + 4] = 0.5 * dt * q3;
    ekf->F->r[2 * 10 + 5] = 0.5 * dt * q0;
    ekf->F->r[2 * 10 + 6] = -0.5 * dt * q1;
    ekf->F->r[2 * 10 + 7] = 0;
    ekf->F->r[2 * 10 + 8] = 0;
    ekf->F->r[2 * 10 + 9] = 0;

    ekf->F->r[3 * 10 + 0] = 0.5 * dt * wz;
    ekf->F->r[3 * 10 + 1] = 0.5 * dt * wy;
    ekf->F->r[3 * 10 + 2] = -0.5 * dt * wx;
    ekf->F->r[3 * 10 + 3] = 1;
    ekf->F->r[3 * 10 + 4] = -0.5 * dt * q2;
    ekf->F->r[3 * 10 + 5] = 0.5 * dt * q1;
    ekf->F->r[3 * 10 + 6] = 0.5 * dt * q0;
    ekf->F->r[3 * 10 + 7] = 0;
    ekf->F->r[3 * 10 + 8] = 0;
    ekf->F->r[3 * 10 + 9] = 0;

    ekf->F->r[4 * 10 + 0] = 0;
    ekf->F->r[4 * 10 + 1] = 0;
    ekf->F->r[4 * 10 + 2] = 0;
    ekf->F->r[4 * 10 + 3] = 0;
    ekf->F->r[4 * 10 + 4] = 0;
    ekf->F->r[4 * 10 + 5] = 0;
    ekf->F->r[4 * 10 + 6] = 0;
    ekf->F->r[4 * 10 + 7] = -1;
    ekf->F->r[4 * 10 + 8] = 0;
    ekf->F->r[4 * 10 + 9] = 0;

    ekf->F->r[5 * 10 + 0] = 0;
    ekf->F->r[5 * 10 + 1] = 0;
    ekf->F->r[5 * 10 + 2] = 0;
    ekf->F->r[5 * 10 + 3] = 0;
    ekf->F->r[5 * 10 + 4] = 0;
    ekf->F->r[5 * 10 + 5] = 0;
    ekf->F->r[5 * 10 + 6] = 0;
    ekf->F->r[5 * 10 + 7] = 0;
    ekf->F->r[5 * 10 + 8] = -1;
    ekf->F->r[5 * 10 + 9] = 0;

    ekf->F->r[6 * 10 + 0] = 0;
    ekf->F->r[6 * 10 + 1] = 0;
    ekf->F->r[6 * 10 + 2] = 0;
    ekf->F->r[6 * 10 + 3] = 0;
    ekf->F->r[6 * 10 + 4] = 0;
    ekf->F->r[6 * 10 + 5] = 0;
    ekf->F->r[6 * 10 + 6] = 0;
    ekf->F->r[6 * 10 + 7] = 0;
    ekf->F->r[6 * 10 + 8] = 0;
    ekf->F->r[6 * 10 + 9] = -1;

    ekf->F->r[7 * 10 + 0] = 0;
    ekf->F->r[7 * 10 + 1] = 0;
    ekf->F->r[7 * 10 + 2] = 0;
    ekf->F->r[7 * 10 + 3] = 0;
    ekf->F->r[7 * 10 + 4] = 0;
    ekf->F->r[7 * 10 + 5] = 0;
    ekf->F->r[7 * 10 + 6] = 0;
    ekf->F->r[7 * 10 + 7] = 1;
    ekf->F->r[7 * 10 + 8] = 0;
    ekf->F->r[7 * 10 + 9] = 0;

    ekf->F->r[8 * 10 + 0] = 0;
    ekf->F->r[8 * 10 + 1] = 0;
    ekf->F->r[8 * 10 + 2] = 0;
    ekf->F->r[8 * 10 + 3] = 0;
    ekf->F->r[8 * 10 + 4] = 0;
    ekf->F->r[8 * 10 + 5] = 0;
    ekf->F->r[8 * 10 + 6] = 0;
    ekf->F->r[8 * 10 + 7] = 0;
    ekf->F->r[8 * 10 + 8] = 1;
    ekf->F->r[8 * 10 + 9] = 0;

    ekf->F->r[9 * 10 + 0] = 0;
    ekf->F->r[9 * 10 + 1] = 0;
    ekf->F->r[9 * 10 + 2] = 0;
    ekf->F->r[9 * 10 + 3] = 0;
    ekf->F->r[9 * 10 + 4] = 0;
    ekf->F->r[9 * 10 + 5] = 0;
    ekf->F->r[9 * 10 + 6] = 0;
    ekf->F->r[9 * 10 + 7] = 0;
    ekf->F->r[9 * 10 + 8] = 0;
    ekf->F->r[9 * 10 + 9] = 1;

    //ekf->P = ekf->F * ekf->P * ekf->F.transpose() + ekf->Q;
    Mat *temp_mat1 = mat_alloc(10, 10);
    Mat *temp_mat2 = mat_alloc(10, 10);
    Mat *F_trans =  mat_alloc(10, 10);
    mat_transpose(ekf->F, F_trans);
    mat_mult(ekf->P, F_trans, temp_mat1);
    mat_mult(ekf->F, temp_mat1, temp_mat2);
    mat_add(temp_mat2, ekf->Q, ekf->P);
    mat_free(temp_mat1);
    mat_free(temp_mat2);
    mat_free(F_trans);

    q0 = ekf->x->r[0];
    q1 = ekf->x->r[1];
    q2 = ekf->x->r[2];
    q3 = ekf->x->r[3];

    ekf->Rot->r[0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    ekf->Rot->r[1] = 2 * (q1 * q2 - q0 * q3);
    ekf->Rot->r[2] = 2 * (q0 * q2 + q1 * q3);

    ekf->Rot->r[3] = 2 * (q1 * q2 + q0 * q3);
    ekf->Rot->r[4] = (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3);
    ekf->Rot->r[5] = 2 * (q2 * q3 - q0 * q1);

    ekf->Rot->r[6] = 2 * (q1 * q3 - q0 * q2);
    ekf->Rot->r[7] = 2 * (q0 * q1 + q2 * q3);
    ekf->Rot->r[8] = (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    mat_transpose(ekf->Rot, ekf->Rot_inv);

    quat_from_vec4(ekf->x, q);


    Vec *rev_g = vec_alloc(3);
    Vec *z_acc = vec_alloc(3);
    Vec *rpy = vec_alloc(3);
    rev_g->r[2] = -1;
    mat_vec_mult(ekf->Rot_inv, rev_g, z_acc);
    quat_to_rpy(q, rpy);
    float zyaw = rpy->r[2];
    ekf->z->r[0] = z_acc->r[0];
    ekf->z->r[1] = z_acc->r[1];
    ekf->z->r[2] = z_acc->r[2];
    ekf->z->r[3] = zyaw;
    vec_free(rev_g);
    vec_free(z_acc);
    vec_free(rpy);

    float dhm_dqw =
        (2 * q3 * (1 - 2 * (q2 * q2 + q3 * q3))) /
        (4 * (q1 * q2 + q0 * q3) * (q1 * q2 + q0 * q3) +
         (1 - 2 * (q2 * q2 + q3 * q3)) * (1 - 2 * (q2 * q2 + q3 * q3)));
    float dhm_dqi =
        (2 * q2 * (1 - 2 * (q2 * q2 + q3 * q3))) /
        (4 * (q1 * q2 + q0 * q3) * (q1 * q2 + q0 * q3) +
         (1 - 2 * (q2 * q2 + q3 * q3)) * (1 - 2 * (q2 * q2 + q3 * q3)));
    float dhm_dqj =
        (2 * (q1 + 2 * q1 * q2 * q2 + 4 * q0 * q2 * q3 - 2 * q1 * q3 * q3)) /
        (1 + 4 * q2 * q2 * q2 * q2 + 8 * q0 * q1 * q2 * q3 +
         4 * (-1 + q0 * q0) * q3 * q3 + 4 * q3 * q3 * q3 * q3 +
         4 * q2 * q2 * (-1 + q1 * q1 + 2 * q3 * q3));
    float dhm_dqk = (8 * q1 * q2 * q3 + q0 * (2 - 4 * q2 * q2 + 4 * q3 * q3)) /
                     (1 + 4 * q2 * q2 * q2 * q2 + 8 * q0 * q1 * q2 * q3 +
                      4 * (-1 + q0 * q0) * q3 * q3 + 4 * q3 * q3 * q3 * q3 +
                      4 * q2 * q2 * (-1 + q1 * q1 + 2 * q3 * q3));

    ekf->H->r[0 * 10 + 0] =  2 * q2;
    ekf->H->r[0 * 10 + 1] = -2 * q3;
    ekf->H->r[0 * 10 + 2] = 2 * q0;
    ekf->H->r[0 * 10 + 3] = -2 * q1;
    ekf->H->r[0 * 10 + 4] = 0;
    ekf->H->r[0 * 10 + 5] = 0;
    ekf->H->r[0 * 10 + 6] = 0;
    ekf->H->r[0 * 10 + 7] = 0;
    ekf->H->r[0 * 10 + 8] = 0;
    ekf->H->r[0 * 10 + 9] = 0;

    ekf->H->r[1 * 10 + 0] = -2 * q1;
    ekf->H->r[1 * 10 + 1] = -2 * q0;
    ekf->H->r[1 * 10 + 2] = -2 * q3;
    ekf->H->r[1 * 10 + 3] = -2 * q2;
    ekf->H->r[1 * 10 + 4] = 0;
    ekf->H->r[1 * 10 + 5] = 0;
    ekf->H->r[1 * 10 + 6] = 0;
    ekf->H->r[1 * 10 + 7] = 0;
    ekf->H->r[1 * 10 + 8] = 0;
    ekf->H->r[1 * 10 + 9] = 0;

    ekf->H->r[2 * 10 + 0] = -2 * q0;
    ekf->H->r[2 * 10 + 1] = 2 * q1;
    ekf->H->r[2 * 10 + 2] = 2 * q2;
    ekf->H->r[2 * 10 + 3] = -2 * q3;
    ekf->H->r[2 * 10 + 4] = 0;
    ekf->H->r[2 * 10 + 5] = 0;
    ekf->H->r[2 * 10 + 6] = 0;
    ekf->H->r[2 * 10 + 7] = 0;
    ekf->H->r[2 * 10 + 8] = 0;
    ekf->H->r[2 * 10 + 9] = 0;

    ekf->H->r[3 * 10 + 0] = dhm_dqw;
    ekf->H->r[3 * 10 + 1] = dhm_dqi;
    ekf->H->r[3 * 10 + 2] = dhm_dqj;
    ekf->H->r[3 * 10 + 3] = dhm_dqk;
    ekf->H->r[3 * 10 + 4] = 0;
    ekf->H->r[3 * 10 + 5] = 0;
    ekf->H->r[3 * 10 + 6] = 0;
    ekf->H->r[3 * 10 + 7] = 0;
    ekf->H->r[3 * 10 + 8] = 0;
    ekf->H->r[3 * 10 + 9] = 0;

    quat_free(q);
}

void EKF_update(EKF *ekf) {
    Vec *v =  vec_alloc(4);
    vec_sub(ekf->y, ekf->z, v);
    if (v->r[3] > M_PI) {
      v->r[3] -= 2 * M_PI;
    } else if (v->r[3] < -M_PI) {
        v->r[3] += 2 * M_PI;
    }

    //Mat<4,4> S = this->H * this->P * this->H.transpose() + this->R;
    Mat *S = mat_alloc(4, 4);
    Mat *S_inv = mat_alloc(4, 4);
    Mat *H_trans= mat_alloc(10, 4);
    Mat *tmp1 = mat_alloc(10, 4);
    Mat *tmp2 = mat_alloc(4, 4);
    mat_transpose(ekf->H, H_trans);
    mat_mult(ekf->P, H_trans, tmp1);
    mat_mult(ekf->H, tmp1, tmp2);
    mat_add(tmp2, ekf->R, S);

    mat_free(tmp2);

    //this->K = this->P * this->H.transpose() * S.inverse();
    mat_inverse(S, S_inv);
    mat_mult(H_trans, S_inv, tmp1);
    mat_mult(ekf->P, tmp1, ekf->K);

    mat_free(H_trans);
    mat_free(S);
    mat_free(S_inv);

    //this->x = this->x + this->K * v;
    Vec *tmp = vec_alloc(10);
    mat_vec_mult(ekf->K, v, tmp);
    vec_add(ekf->x, tmp, ekf->x);
    vec_free(tmp);

    Quat *q = quat_alloc();
    quat_from_vec4(ekf->x, q);
    quat_normalize(q);
    ekf->x->r[0] = q->q;
    ekf->x->r[1] = q->i;
    ekf->x->r[2] = q->j;
    ekf->x->r[3] = q->k;

    //this->P = (Mat<10,10>().identity() - (this->K * this->H)) * this->P;
    Mat *i10 = mat_alloc(10, 10);
    Mat *tmp3 = mat_alloc(10, 10);
    Mat *tmp4 = mat_alloc(10, 10);
    mat_identity(i10);
    mat_mult(ekf->K, ekf->H, tmp3);
    mat_sub(i10, tmp3, tmp4);
    mat_mult(tmp4, ekf->P, tmp3);

    for (int i = 0; i < 100; ++i) {
        ekf->P->r[i] = tmp3->r[i];
    }

    ekf->attitude->q = q->q;
    ekf->attitude->i = q->i;
    ekf->attitude->j = q->j;
    ekf->attitude->k = q->k;

    vec_free(v);
    quat_free(q);
    mat_free(i10);
    mat_free(tmp3);
    mat_free(tmp4);
}

int gyro_cnt = 0;
int acc_cnt = 0;
int mag_cnt = 0;
EKF ekf;
Vec *gyro[100];
Vec *mag[100];
Vec *acc[100];
int has_init = 0;

volatile void attitude_thread() {
    //    uint64_t next_mag_time = now();
    //    uint32_t last_time = now_high_accuracy();
    //    while (1) {
    //        volatile uint64_t next_time = now() + 3 * MILLISECONDS;
    //        if (next_mag_time < now()) { // we only get this every 50 milliseconds, so setting the value more ofter just wastes computation
    //            next_mag_time = now() + 50 * MILLISECONDS;
    //            ekf.update_mag(LSM9DS1_mag);
    //
    //            if (mag_cnt < 100) {
    //                mag[mag_cnt] = LSM9DS1_mag;
    //                mag_cnt++;
    //            }
    //        }
    //
    //        ekf.update_acc(LSM9DS1_acc);
    //        if (has_init) {
    //            uint32_t start = now_high_accuracy();
    //            ekf.predict(LSM9DS1_gyro, 0.03);//(float) (now_high_accuracy() - last_time) / 1e6);
    //            last_time = now_high_accuracy();
    //            ekf.update();
    //            ekf.attitude.print_bare();
    //            //os_printf("dt: %f [ms]\n", (float) (now_high_accuracy() - start));
    //        }
    //
    //        if (acc_cnt < 100) {
    //            acc[acc_cnt] = LSM9DS1_acc;
    //            acc_cnt++;
    //        }
    //        if (gyro_cnt < 100) {
    //            gyro[gyro_cnt] = LSM9DS1_gyro;
    //            gyro_cnt++;
    //        }
    //        if (gyro_cnt == 100 && mag_cnt == 100 && acc_cnt == 100 && !has_init) {
    //            ekf.init(gyro, acc, mag);
    //            has_init = 1;
    //        }
    //        sleep_until(next_time);
    //    }
}
