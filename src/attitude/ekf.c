#include "ekf.h"
#include <math.h>
#include <stdint.h>
// Quaternions
QUAT_ALLOC_STATIC(attitude_static);
QUAT_ALLOC_STATIC(q_static);
QUAT_ALLOC_STATIC(w_static);
QUAT_ALLOC_STATIC(q_temp_static);

// Vectors
VEC_ALLOC_STATIC(bias_static, 3);
VEC_ALLOC_STATIC(x_static, 10);
VEC_ALLOC_STATIC(y_static, 4);
VEC_ALLOC_STATIC(z_static, 4);
VEC_ALLOC_STATIC(rev_g_static, 3);
VEC_ALLOC_STATIC(z_acc_static, 3);
VEC_ALLOC_STATIC(rpy_static, 3);
VEC_ALLOC_STATIC(v_static, 4);
VEC_ALLOC_STATIC(tmp_static, 10);
VEC_ALLOC_STATIC(m_static, 3);
VEC_ALLOC_STATIC(mn_static, 3);

// Matrices
MAT_ALLOC_STATIC(P_static, 10, 10);
MAT_ALLOC_STATIC(F_static, 10, 10);
MAT_ALLOC_STATIC(Q_static, 10, 10);
MAT_ALLOC_STATIC(H_static, 4, 10);
MAT_ALLOC_STATIC(K_static, 10, 4);
MAT_ALLOC_STATIC(R_static, 4, 4);
MAT_ALLOC_STATIC(Rot_static, 3, 3);
MAT_ALLOC_STATIC(Rot_inv_static, 3, 3);

MAT_ALLOC_STATIC(temp_mat1_static, 10, 10);
MAT_ALLOC_STATIC(temp_mat2_static, 10, 10);
MAT_ALLOC_STATIC(F_trans_static, 10, 10);

MAT_ALLOC_STATIC(S_static, 4, 4);
MAT_ALLOC_STATIC(S_inv_static, 4, 4);
MAT_ALLOC_STATIC(H_trans_static, 10, 4);
MAT_ALLOC_STATIC(tmp1_static, 10, 4);
MAT_ALLOC_STATIC(tmp2_static, 4, 4);

MAT_ALLOC_STATIC(i10_static, 10, 10);
MAT_ALLOC_STATIC(tmp3_static, 10, 10);
MAT_ALLOC_STATIC(tmp4_static, 10, 10);

EKF ekf_static = {
    .attitude = &attitude_static,
    .bias = &bias_static,
    .x = &x_static,
    .y = &y_static,
    .z = &z_static,
    .P = &P_static,
    .F = &F_static,
    .Q = &Q_static,
    .H = &H_static,
    .K = &K_static,
    .R = &R_static,
    .Rot = &Rot_static,
    .Rot_inv = &Rot_inv_static,
    .q = &q_static,
    .w = &w_static,
    .q_temp = &q_temp_static,
    .temp_mat1 = &temp_mat1_static,
    .temp_mat2 = &temp_mat2_static,
    .F_trans = &F_trans_static,
    .rev_g = &rev_g_static,
    .z_acc = &z_acc_static,
    .rpy = &rpy_static,
    .v = &v_static,
    .S = &S_static,
    .S_inv = &S_inv_static,
    .H_trans = &H_trans_static,
    .tmp1 = &tmp1_static,
    .tmp2 = &tmp2_static,
    .tmp = &tmp_static,
    .i10 = &i10_static,
    .tmp3 = &tmp3_static,
    .tmp4 = &tmp4_static,
    .m = &m_static,
    .mn = &mn_static
};


void EKF_init(EKF *ekf, Vec **gyro, Vec **acc, Vec **mag) {
    int num_init = 20;
    Vec *mean_gyro = vec_alloc(3);
    Vec *mean_acc = vec_alloc(3);
    Vec *mean_mag = vec_alloc(3);

    float init_div = 1. / num_init;
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
    Quat *q_init = quat_alloc();
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
    mat_transpose(Fu, Fu_trans);

    float q0 = ekf->x->r[0];
    float q1 = ekf->x->r[1];
    float q2 = ekf->x->r[2];
    float q3 = ekf->x->r[3];

    ekf->Rot->r[0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    ekf->Rot->r[1] = 2 * (q1 * q2 - q0 * q3);
    ekf->Rot->r[2] = 2 * (q0 * q2 + q1 * q3);

    ekf->Rot->r[3] = 2 * (q1 * q2 - q0 * q3);
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
    vec_free(mean_gyro);
    vec_free(mean_acc);
    vec_free(mean_mag);
    quat_free(q_init);
    vec_free(gyro_sum);
    vec_free(acc_sum);
}

void EKF_update_acc(EKF *ekf, Vec *acc) {
    ekf->y->r[0] = acc->r[0];
    ekf->y->r[1] = acc->r[1];
    ekf->y->r[2] = acc->r[2];
}

void EKF_update_mag(EKF *ekf, Vec *mag, Vec *acc) {
    mat_vec_mult(ekf->Rot, mag, ekf->m);
    ekf->m->r[2] = 0;

    //m =  ekf->Rot_inv * m;
    mat_vec_mult(ekf->Rot_inv, ekf->m, ekf->mn);

    float yaw = atan2(- ekf->mn->r[1], ekf->mn->r[0]);
    ekf->y->r[3] = yaw;
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

    quat_from_vec4(ekf->x, ekf->q);

    ekf->w->q = 0;
    ekf->w->i = wx;
    ekf->w->j = wy;
    ekf->w->k = wz;

    //q = q + q * w * 0.5 * dt;
    quat_mult(ekf->q, ekf->w, ekf->q_temp);
    quat_scalar_mult(ekf->q_temp, 0.5f * dt);
    quat_add(ekf->q, ekf->q_temp, ekf->q);
    quat_normalize(ekf->q);

    ekf->x->r[0] = ekf->q->q;
    ekf->x->r[1] = ekf->q->i;
    ekf->x->r[2] = ekf->q->j;
    ekf->x->r[3] = ekf->q->k;
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
    mat_transpose(ekf->F, ekf->F_trans);
    mat_mult(ekf->F, ekf->P, ekf->temp_mat1);
    mat_mult(ekf->temp_mat1, ekf->F_trans, ekf->temp_mat2);
    mat_add(ekf->temp_mat2, ekf->Q, ekf->P);

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

    quat_from_vec4(ekf->x, ekf->q);

    ekf->rev_g->r[2] = -1;
    mat_vec_mult(ekf->Rot_inv, ekf->rev_g, ekf->z_acc);
    quat_to_rpy(ekf->q, ekf->rpy);
    float zyaw = ekf->rpy->r[2];
    ekf->z->r[0] = ekf->z_acc->r[0];
    ekf->z->r[1] = ekf->z_acc->r[1];
    ekf->z->r[2] = ekf->z_acc->r[2];
    ekf->z->r[3] = zyaw;

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
}

void EKF_update(EKF *ekf) {
    vec_sub(ekf->y, ekf->z, ekf->v);
    if (ekf->v->r[3] > M_PI) {
      ekf->v->r[3] -= 2 * M_PI;
    } else if (ekf->v->r[3] < -M_PI) {
        ekf->v->r[3] += 2 * M_PI;
    }

    //Mat<4,4> S = this->H * this->P * this->H.transpose() + this->R;
    mat_transpose(ekf->H, ekf->H_trans);
    mat_mult(ekf->P, ekf->H_trans, ekf->tmp1);
    mat_mult(ekf->H, ekf->tmp1, ekf->tmp2);
    mat_add(ekf->tmp2, ekf->R, ekf->S);

    //this->K = this->P * this->H.transpose() * S.inverse();
    mat_inverse(ekf->S, ekf->S_inv);
    mat_mult(ekf->H_trans, ekf->S_inv, ekf->tmp1);
    mat_mult(ekf->P, ekf->tmp1, ekf->K);

    //this->x = this->x + this->K * v;
    mat_vec_mult(ekf->K, ekf->v, ekf->tmp);
    vec_add(ekf->x, ekf->tmp, ekf->x);

    quat_from_vec4(ekf->x, ekf->q);
    quat_normalize(ekf->q);
    ekf->x->r[0] = ekf->q->q;
    ekf->x->r[1] = ekf->q->i;
    ekf->x->r[2] = ekf->q->j;
    ekf->x->r[3] = ekf->q->k;

    //this->P = (Mat<10,10>().identity() - (this->K * this->H)) * this->P;
    mat_identity(ekf->i10);
    mat_mult(ekf->K, ekf->H, ekf->tmp3);
    mat_sub(ekf->i10, ekf->tmp3, ekf->tmp4);
    mat_mult(ekf->tmp4, ekf->P, ekf->tmp3);

    for (int i = 0; i < 100; ++i) {
        ekf->P->r[i] = ekf->tmp3->r[i];
    }

    ekf->attitude->q = ekf->q->q;
    ekf->attitude->i = ekf->q->i;
    ekf->attitude->j = ekf->q->j;
    ekf->attitude->k = ekf->q->k;
}

int gyro_cnt = 0;
int acc_cnt = 0;
int mag_cnt = 0;
Vec *gyro[20];
Vec *mag[20];
Vec *acc[20];

struct EKF *ekf;
volatile void attitude_thread() {
    ekf = &ekf_static;
    int has_init = 0;
    for (int i = 0; i < 20; ++i) {
        gyro[i] = vec_alloc(3);
        acc[i] = vec_alloc(3);
        mag[i] = vec_alloc(3);
    }

    uint64_t next_mag_time = now();
    uint32_t last_time = now_high_accuracy();
    while (1) {
        volatile uint64_t next_time = now() + 3 * MILLISECONDS;
        if (next_mag_time < now()) { // we only get this every 50 milliseconds, so setting the value
                                     // more ofter just wastes computation
            next_mag_time = now() + 50 * MILLISECONDS;
            EKF_update_mag(ekf, &LSM9DS1_mag, &LSM9DS1_acc);

                if (mag_cnt < 20) {
                mag[mag_cnt]->r[0] = LSM9DS1_mag.r[0];
                mag[mag_cnt]->r[1] = LSM9DS1_mag.r[1];
                mag[mag_cnt]->r[2] = LSM9DS1_mag.r[2];
                mag_cnt++;
            }
        }

        if (has_init) {
            EKF_update_acc(ekf, &LSM9DS1_acc);
            uint32_t start = now_high_accuracy();
            EKF_predict(ekf, &LSM9DS1_gyro, 0.03); //(float) (now_high_accuracy() - last_time) / 1e6);
            last_time = now_high_accuracy();
            EKF_update(ekf);
            quat_print(ekf->attitude);
            // os_printf("dt: %f [ms]\n", (float) (now_high_accuracy() - start));
        }

        if (acc_cnt < 20) {
            acc[acc_cnt]->r[0] = LSM9DS1_acc.r[0];
            acc[acc_cnt]->r[1] = LSM9DS1_acc.r[1];
            acc[acc_cnt]->r[2] = LSM9DS1_acc.r[2];
            acc_cnt++;
        }
        if (gyro_cnt < 20) {
            gyro[gyro_cnt]->r[0] = LSM9DS1_gyro.r[0];
            gyro[gyro_cnt]->r[1] = LSM9DS1_gyro.r[1];
            gyro[gyro_cnt]->r[2] = LSM9DS1_gyro.r[2];
            gyro_cnt++;
        }

        if (gyro_cnt == 20 && mag_cnt == 20 && acc_cnt == 20 && !has_init) {
            EKF_init(ekf, gyro, acc, mag);
            has_init = 1;
            for (int i = 0; i < 20; ++i) {
                vec_free(gyro[i]);
                vec_free(mag[i]);
                vec_free(acc[i]);
            }
        }

        sleep_until(next_time);
    }
}
