#include "ekf.h"
#include <math.h>
#include <stdint.h>
// Quaternions
QUAT_ALLOC_STATIC(attitude_static);
QUAT_ALLOC_STATIC(q_static);

// Vectors
VEC_ALLOC_STATIC(bias_static, 3);
VEC_ALLOC_STATIC(x_static, 7);
VEC_ALLOC_STATIC(h_static, 4);
VEC_ALLOC_STATIC(z_static, 4);
VEC_ALLOC_STATIC(y_static, 4);
VEC_ALLOC_STATIC(v_static, 4);
VEC_ALLOC_STATIC(tmp_static, 7);

// Matrices
MAT_ALLOC_STATIC(P_static, 7, 7);
MAT_ALLOC_STATIC(F_static, 7, 7);
MAT_ALLOC_STATIC(Q_static, 7, 7);
MAT_ALLOC_STATIC(H_static, 4, 7);
MAT_ALLOC_STATIC(K_static, 7, 4);
MAT_ALLOC_STATIC(R_static, 4, 4);
MAT_ALLOC_STATIC(Rot_static, 3, 3);
MAT_ALLOC_STATIC(Rot_inv_static, 3, 3);

MAT_ALLOC_STATIC(temp_mat1_static, 7, 7);
MAT_ALLOC_STATIC(temp_mat2_static, 7, 7);
MAT_ALLOC_STATIC(F_trans_static, 7, 7);

MAT_ALLOC_STATIC(S_static, 4, 4);
MAT_ALLOC_STATIC(S_inv_static, 4, 4);
MAT_ALLOC_STATIC(H_trans_static, 7, 4);
MAT_ALLOC_STATIC(tmp1_static, 7, 4);
MAT_ALLOC_STATIC(tmp2_static, 4, 4);

MAT_ALLOC_STATIC(i7_static, 7, 7);
MAT_ALLOC_STATIC(tmp3_static, 7, 7);
MAT_ALLOC_STATIC(tmp4_static, 7, 7);

EKF ekf_static = {
    .attitude = &attitude_static,
    .bias = &bias_static,
    .x = &x_static,
    .h = &h_static,
    .z = &z_static,
    .y = &y_static,
    .P = &P_static,
    .F = &F_static,
    .Q = &Q_static,
    .H = &H_static,
    .K = &K_static,
    .R = &R_static,
    .Rot = &Rot_static,
    .Rot_inv = &Rot_inv_static,
    .q = &q_static,
    .temp_mat1 = &temp_mat1_static,
    .temp_mat2 = &temp_mat2_static,
    .F_trans = &F_trans_static,
    .S = &S_static,
    .S_inv = &S_inv_static,
    .H_trans = &H_trans_static,
    .tmp1 = &tmp1_static,
    .tmp2 = &tmp2_static,
    .tmp = &tmp_static,
    .i7 = &i7_static,
    .tmp3 = &tmp3_static,
    .tmp4 = &tmp4_static,
};

void rp_from_acc(Vec *acc, float* roll, float* pitch) {
    float m_a_c_x = acc->r[0];
    float m_a_c_y = acc->r[1];
    float m_a_c_z = acc->r[2];

    *roll = atan2(m_a_c_y, m_a_c_z);
    *pitch = atan(- m_a_c_x / sqrt(m_a_c_y * m_a_c_y + m_a_c_z * m_a_c_z));
}

void EKF_update_acc(EKF *ekf, Vec *acc) {
    vec_copy(acc, ekf->z);
}

void EKF_update_mag(EKF *ekf, Vec *mag, Vec *acc) {
    float roll, pitch;
    float mm_c_x, mm_c_y;
    float m_x = mag->r[0];
    float m_y = mag->r[1];
    float m_z = mag->r[2];

    rp_from_acc(acc, &roll, &pitch);

    mm_c_x = m_x * cos(pitch) + m_z * sin(pitch);
    mm_c_y = m_x * sin(roll) * sin(pitch) + m_y * cos(roll) - m_z * sin(roll) * cos(pitch);

    float yaw = atan2(- mm_c_y, mm_c_x);
    ekf->z->r[3] = yaw;
}

void predict_state(EKF *ekf, Vec *gyro, float dt) {
    float q_q_k = ekf->attitude->q;
    float q_i_k = ekf->attitude->i;
    float q_j_k = ekf->attitude->j;
    float q_k_k = ekf->attitude->k;
    float wx = gyro->r[0] - ekf->bias->r[0];
    float wy = gyro->r[1] - ekf->bias->r[1];
    float wz = gyro->r[2] - ekf->bias->r[2];

    float q_q_kp1 = q_q_k + 0.5 *  dt * (- wx * q_i_k - wy * q_j_k - wz * q_k_k);
    float q_i_kp1 = q_q_k + 0.5 *  dt * (wx * q_q_k - wz * q_j_k + wy * q_k_k);
    float q_j_kp1 = q_q_k + 0.5 *  dt * (wy * q_q_k + wz * q_i_k - wx * q_k_k);
    float q_k_kp1 = q_q_k + 0.5 *  dt * (- wz * q_q_k - wy * q_i_k - wx * q_j_k);

    float norm_div  = 1 / sqrt(q_q_kp1 * q_q_kp1 + q_i_kp1 * q_i_kp1 + q_j_kp1 * q_j_kp1 + q_k_kp1 * q_k_kp1);
    q_q_kp1 *= norm_div;
    q_i_kp1 *= norm_div;
    q_j_kp1 *= norm_div;
    q_k_kp1 *= norm_div;

    ekf->x->r[0] = q_q_k;
    ekf->x->r[1] = q_i_k;
    ekf->x->r[2] = q_j_k;
    ekf->x->r[3] = q_k_k;

    ekf->x->r[4] = ekf->x->r[4];
    ekf->x->r[5] = ekf->x->r[5];
    ekf->x->r[6] = ekf->x->r[6];
}

void F_jacobian(EKF *ekf, Vec *gyro, float dt) {
    float wx = gyro->r[0] - ekf->bias->r[0];
    float wy = gyro->r[1] - ekf->bias->r[1];
    float wz = gyro->r[2] - ekf->bias->r[2];

    ekf->F->r[0 + 7 * 0] = 1;
    ekf->F->r[1 + 7 * 0] = - 0.5 * dt * wx;
    ekf->F->r[2 + 7 * 0] = - 0.5 * dt * wy;
    ekf->F->r[3 + 7 * 0] = - 0.5 * dt * wz;

    ekf->F->r[0 + 7 * 1] = 0.5 * dt * wx;
    ekf->F->r[1 + 7 * 1] = 1;
    ekf->F->r[2 + 7 * 1] = - 0.5 * dt * wz;
    ekf->F->r[3 + 7 * 1] = 0.5 * dt * wy;

    ekf->F->r[0 + 7 * 2] = 0.5 * dt * wy;
    ekf->F->r[1 + 7 * 2] = - 0.5 * dt * wz;
    ekf->F->r[2 + 7 * 2] = 1;
    ekf->F->r[3 + 7 * 2] = - 0.5 * dt * wx;

    ekf->F->r[0 + 7 * 3] = - 0.5 * dt * wz;
    ekf->F->r[1 + 7 * 3] = - 0.5 * dt * wy;
    ekf->F->r[2 + 7 * 3] = 0.5 * dt * wx;
    ekf->F->r[3 + 7 * 3] = 1;

    float qw = ekf->attitude->q;
    float qi = ekf->attitude->i;
    float qj = ekf->attitude->j;
    float qk = ekf->attitude->k;

    ekf->F->r[4 + 7 * 0] = - 0.5 * dt * qi;
    ekf->F->r[5 + 7 * 0] = - 0.5 * dt * qj;
    ekf->F->r[6 + 7 * 0] = - 0.5 * dt * qk;

    ekf->F->r[4 + 7 * 1] = 0.5 * dt * qw;
    ekf->F->r[5 + 7 * 1] = 0.5 * dt * qk;
    ekf->F->r[6 + 7 * 1] = - 0.5 * dt * qj;

    ekf->F->r[4 + 7 * 2] = - 0.5 * dt * qk;
    ekf->F->r[5 + 7 * 2] = - 0.5 * dt * qw;
    ekf->F->r[6 + 7 * 2] = 0.5 * dt * qi;

    ekf->F->r[4 + 7 * 3] = 0.5 * dt * qj;
    ekf->F->r[5 + 7 * 3] = - 0.5 * dt * qi;
    ekf->F->r[6 + 7 * 3] = - 0.5 * dt * qw;

    ekf->F->r[4 + 7 * 4] = 1;
    ekf->F->r[5 + 7 * 5] = 1;
    ekf->F->r[6 + 7 * 6] = 1;
}

void EKF_predict(EKF *ekf, Vec *gyro, float dt) {
    predict_state(ekf, gyro, dt);

    F_jacobian(ekf, gyro, dt);
    mat_transpose(ekf->F, ekf->F_trans);
    mat_mult(ekf->F, ekf->P, ekf->temp_mat1);
    mat_mult(ekf->temp_mat1, ekf->F_trans, ekf->temp_mat2);
    mat_add(ekf->temp_mat2, ekf->Q, ekf->P);
}

void h_prediction(EKF *ekf) {
    float qw = ekf->x->r[0];
    float qi = ekf->x->r[1];
    float qj = ekf->x->r[2];
    float qk = ekf->x->r[3];

    ekf->h->r[0] = 2 * (qi * qk - qw * qj);
    ekf->h->r[1] = 2 * (qj * qk + qw * qi);
    ekf->h->r[2] = qw * qw - qi * qi - qj * qj + qk * qk;
    ekf->h->r[3] = atan2(2 * (qw * qk + qi * qj, 1 - 2 * qj * qj - 2 * qk * qk));
}

void limit_yaw(float *yaw) {
    if (*yaw > M_PI) {
        *yaw -= 2 * M_PI;
    } else if (*yaw < -M_PI) {
        *yaw += 2 * M_PI;
    }
}

void H_jacobian(EKF *ekf) {
    float qw = ekf->x->r[0];
    float qi = ekf->x->r[1];
    float qj = ekf->x->r[2];
    float qk = ekf->x->r[3];

    float A = 2 * (qw * qk + qi * qj);
    float B = 1 - 2 * qj * qj - 2 * qk * qk;
    float div_sqare = 1 / (A * A + B * B);

    ekf->H->r[0 + 7 * 0] = - 2 * qj;
    ekf->H->r[1 + 7 * 0] = 2 * qk;
    ekf->H->r[2 + 7 * 0] = - 2 * qw;
    ekf->H->r[3 + 7 * 0] = 2 * qi;

    ekf->H->r[0 + 7 * 1] = 2 * qi;
    ekf->H->r[1 + 7 * 1] = 2 * qw;
    ekf->H->r[2 + 7 * 1] = 2 * qk;
    ekf->H->r[3 + 7 * 1] = 2 * qj;

    ekf->H->r[0 + 7 * 2] = 2 * qw;
    ekf->H->r[1 + 7 * 2] = - 2 * qi;
    ekf->H->r[2 + 7 * 2] = - 2 * qj;
    ekf->H->r[3 + 7 * 2] = 2 * qk;

    ekf->H->r[0 + 7 * 3] = B * div_sqare * 2 * qk;
    ekf->H->r[1 + 7 * 3] = B * div_sqare * 2 * qj;
    ekf->H->r[2 + 7 * 3] = (2 * B * qi - 4 * A * qj) * div_sqare;
    ekf->H->r[3 + 7 * 3] = (2 * B * qw - 4 * A * qk) * div_sqare;
}

void EKF_update(EKF *ekf) {
    h_prediction(ekf);
    vec_sub(ekf->z, ekf->h, ekf->y);
    limit_yaw(&ekf->y->r[2]);
    H_jacobian(ekf);

    mat_transpose(ekf->H, ekf->H_trans);
    mat_mult(ekf->P, ekf->H_trans, ekf->tmp1);
    mat_mult(ekf->H, ekf->tmp1, ekf->tmp2);
    mat_add(ekf->tmp2, ekf->R, ekf->S);

    //this->K = this->P * this->H.transpose() * S.inverse();
    mat_inverse(ekf->S, ekf->S_inv);
    mat_mult(ekf->H_trans, ekf->S_inv, ekf->tmp1);
    mat_mult(ekf->P, ekf->tmp1, ekf->K);

    //this->x = this->x + this->K * y;
    mat_vec_mult(ekf->K, ekf->y, ekf->tmp);
    vec_add(ekf->x, ekf->tmp, ekf->x);

    quat_from_vec4(ekf->x, ekf->q);
    quat_normalize(ekf->q);
    ekf->x->r[0] = ekf->q->q;
    ekf->x->r[1] = ekf->q->i;
    ekf->x->r[2] = ekf->q->j;
    ekf->x->r[3] = ekf->q->k;

    //this->P = (Mat<10,10>().identity() - (this->K * this->H)) * this->P;
    mat_identity(ekf->i7);
    mat_mult(ekf->K, ekf->H, ekf->tmp3);
    mat_sub(ekf->i7, ekf->tmp3, ekf->tmp4);
    mat_mult(ekf->tmp4, ekf->P, ekf->tmp3);

    for (int i = 0; i < 100; ++i) {
        ekf->P->r[i] = ekf->tmp3->r[i];
    }
    mat_copy(ekf->tmp3, ekf->P);

    ekf->attitude->q = ekf->q->q;
    ekf->attitude->i = ekf->q->i;
    ekf->attitude->j = ekf->q->j;
    ekf->attitude->k = ekf->q->k;
    ekf->bias->r[0] = ekf->x->r[4];
    ekf->bias->r[1] = ekf->x->r[5];
    ekf->bias->r[2] = ekf->x->r[6];
}

void EKF_init_incremental(EKF *ekf, Vec *gyro, Vec *acc, Vec *mag) {

}
