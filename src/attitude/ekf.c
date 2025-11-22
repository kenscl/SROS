#include "ekf.h"
#include <math.h>
#include <stdint.h>
// Quaternions
QUAT_ALLOC_STATIC(attitude_static);
QUAT_ALLOC_STATIC(q_static);

// Vectors
VEC_ALLOC_STATIC(bias_static, 3);
VEC_ALLOC_STATIC(x_static, 7);
VEC_ALLOC_STATIC(h_static, 6);
VEC_ALLOC_STATIC(z_static, 6);
VEC_ALLOC_STATIC(y_static, 6);
VEC_ALLOC_STATIC(v_static, 6);
VEC_ALLOC_STATIC(tmp_static, 7);
VEC_ALLOC_STATIC(acc_refrence_static, 3);
VEC_ALLOC_STATIC(mag_refrence_static, 3);
VEC_ALLOC_STATIC(vtmp_static, 3);

// Matrices
MAT_ALLOC_STATIC(P_static, 7, 7);
MAT_ALLOC_STATIC(F_static, 7, 7);
MAT_ALLOC_STATIC(Q_static, 7, 7);
MAT_ALLOC_STATIC(H_static, 6, 7);
MAT_ALLOC_STATIC(K_static, 7, 6);
MAT_ALLOC_STATIC(R_static, 6, 6);

MAT_ALLOC_STATIC(temp_mat1_static, 7, 7);
MAT_ALLOC_STATIC(temp_mat2_static, 7, 7);
MAT_ALLOC_STATIC(F_trans_static, 7, 7);
MAT_ALLOC_STATIC(W_static, 4, 3);
MAT_ALLOC_STATIC(W_trans_static, 3, 4);

MAT_ALLOC_STATIC(S_static, 6, 6);
MAT_ALLOC_STATIC(S_inv_static, 6, 6);
MAT_ALLOC_STATIC(H_trans_static, 7, 6);
MAT_ALLOC_STATIC(tmp1_static, 7, 6);
MAT_ALLOC_STATIC(tmp2_static, 6, 6);

MAT_ALLOC_STATIC(i7_static, 7, 7);
MAT_ALLOC_STATIC(tmp3_static, 7, 7);
MAT_ALLOC_STATIC(tmp4_static, 7, 7);
MAT_ALLOC_STATIC(rot_static, 3, 3);
MAT_ALLOC_STATIC(rot_inv_static, 3, 3);

EKF ekf = {
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
    .q = &q_static,
    .temp_mat1 = &temp_mat1_static,
    .temp_mat2 = &temp_mat2_static,
    .F_trans = &F_trans_static,
    .W = &W_static,
    .W_trans = &W_trans_static,
    .S = &S_static,
    .S_inv = &S_inv_static,
    .H_trans = &H_trans_static,
    .tmp1 = &tmp1_static,
    .tmp2 = &tmp2_static,
    .tmp = &tmp_static,
    .i7 = &i7_static,
    .tmp3 = &tmp3_static,
    .tmp4 = &tmp4_static,
    .rot = &rot_static,
    .rot_inv =&rot_inv_static,
    .acc_refrence = &acc_refrence_static,
    .mag_refrence = &mag_refrence_static,
    .vtmp = &vtmp_static
};

void EKF_update_acc(EKF *ekf, Vec *acc) {
    ekf->z->r[0] = acc->r[0];
    ekf->z->r[1] = acc->r[1];
    ekf->z->r[2] = acc->r[2];
}

void EKF_update_mag(EKF *ekf, Vec *mag, Vec *acc) {
    ekf->z->r[3] = mag->r[0];
    ekf->z->r[4] = mag->r[1];
    ekf->z->r[5] = mag->r[2];
}

void predict_state(EKF *ekf, Vec *gyro, float dt) {
    float qw = ekf->x->r[0];
    float qi = ekf->x->r[1];
    float qj = ekf->x->r[2];
    float qk = ekf->x->r[3];

    float wx = gyro->r[0]; // - ekf->bias->r[0];
    float wy = gyro->r[1]; // - ekf->bias->r[1];
    float wz = gyro->r[2]; // - ekf->bias->r[2];

    float qwp1 = qw + 0.5f * dt * (-wx * qi - wy * qj - wz * qk);
    float qip1 = qi + 0.5f * dt * ( wx * qw - wy * qk + wz * qj);
    float qjp1 = qj + 0.5f * dt * ( wx * qk + wy * qw - wz * qi);
    float qkp1 = qk + 0.5f * dt * ( - wx * qj + wy * qi + wz * qw);

    float norm = 1; //sqrtf(qwp1*qwp1 + qip1*qip1 + qjp1*qjp1 + qkp1*qkp1);
    qwp1 /= norm;
    qip1 /= norm;
    qjp1 /= norm;
    qkp1 /= norm;

    ekf->x->r[0] = qwp1;
    ekf->x->r[1] = qip1;
    ekf->x->r[2] = qjp1;
    ekf->x->r[3] = qkp1;
}

void F_jacobian(EKF *ekf, Vec *gyro, float dt) {
    ekf->bias->r[0] = 0;
    ekf->bias->r[1] = 0;
    ekf->bias->r[2] = 0;
    ekf->x->r[4] = 0;
    ekf->x->r[5] = 0;
    ekf->x->r[6] = 0;
    float wx = gyro->r[0] - ekf->bias->r[0];
    float wy = gyro->r[1] - ekf->bias->r[1];
    float wz = gyro->r[2] - ekf->bias->r[2];

    mat_fill(ekf->F, 0);
    ekf->F->r[0 + 7 * 0] = 1;
    ekf->F->r[1 + 7 * 0] = - 0.5 * dt * wx;
    ekf->F->r[2 + 7 * 0] = - 0.5 * dt * wy;
    ekf->F->r[3 + 7 * 0] = - 0.5 * dt * wz;

    ekf->F->r[0 + 7 * 1] = 0.5 * dt * wx;
    ekf->F->r[1 + 7 * 1] = 1;
    ekf->F->r[2 + 7 * 1] = 0.5 * dt * wz;
    ekf->F->r[3 + 7 * 1] = - 0.5 * dt * wy;

    ekf->F->r[0 + 7 * 2] = 0.5 * dt * wy;
    ekf->F->r[1 + 7 * 2] = - 0.5 * dt * wz;
    ekf->F->r[2 + 7 * 2] = 1;
    ekf->F->r[3 + 7 * 2] =  0.5 * dt * wx;

    ekf->F->r[0 + 7 * 3] =  0.5 * dt * wz;
    ekf->F->r[1 + 7 * 3] =  0.5 * dt * wy;
    ekf->F->r[2 + 7 * 3] = - 0.5 * dt * wx;
    ekf->F->r[3 + 7 * 3] = 1;

    float qw = ekf->x->r[0];
    float qi = ekf->x->r[1];
    float qj = ekf->x->r[2];
    float qk = ekf->x->r[3];

    //ekf->F->r[4 + 7 * 0] = 0.5 * dt * qi;
    //ekf->F->r[5 + 7 * 0] = 0.5 * dt * qj;
    //ekf->F->r[6 + 7 * 0] = 0.5 * dt * qk;

    //ekf->F->r[4 + 7 * 1] = - 0.5 * dt * qw;
    //ekf->F->r[5 + 7 * 1] = 0.5 * dt * qk;
    //ekf->F->r[6 + 7 * 1] = - 0.5 * dt * qj;

    //ekf->F->r[4 + 7 * 2] = - 0.5 * dt * qk;
    //ekf->F->r[5 + 7 * 2] = - 0.5 * dt * qw;
    //ekf->F->r[6 + 7 * 2] = 0.5 * dt * qi;

    //ekf->F->r[4 + 7 * 3] = 0.5 * dt * qj;
    //ekf->F->r[5 + 7 * 3] = - 0.5 * dt * qi;
    //ekf->F->r[6 + 7 * 3] = - 0.5 * dt * qw;

    //ekf->F->r[4 + 7 * 4] = 1;
    //ekf->F->r[5 + 7 * 5] = 1;
    //ekf->F->r[6 + 7 * 6] = 1;
}

MAT_ALLOC_STATIC(Q_temp, 4, 4);
void Q_prediction(EKF *ekf, float dt) {
    float w = ekf->x->r[0];
    float x = ekf->x->r[1];
    float y = ekf->x->r[2];
    float z = ekf->x->r[3];
    ekf->W->r[0 + 3 * 0] = - x * dt * 0.5;
    ekf->W->r[1 + 3 * 0] = - y * dt * 0.5;
    ekf->W->r[2 + 3 * 0] = - z * dt * 0.5;

    ekf->W->r[0 + 3 * 1] = w * dt * 0.5;
    ekf->W->r[1 + 3 * 1] = -z * dt * 0.5;
    ekf->W->r[2 + 3 * 1] = y * dt * 0.5;

    ekf->W->r[0 + 3 * 2] = z * dt * 0.5;
    ekf->W->r[1 + 3 * 2] = w * dt * 0.5;
    ekf->W->r[2 + 3 * 2] = -x * dt * 0.5;

    ekf->W->r[0 + 3 * 3] = -y * dt * 0.5;
    ekf->W->r[1 + 3 * 3] = x * dt * 0.5;
    ekf->W->r[2 + 3 * 3] = w * dt * 0.5;

    mat_transpose(ekf->W, ekf->W_trans);
    int res = mat_mult(ekf->W, ekf->W_trans, &Q_temp);
    mat_scalar_mult(&Q_temp, ekf->gyro_variance);

    for (int i = 0; i < Q_temp.m; i++) {
        for (int j = 0; j < Q_temp.n; j++) {
            ekf->Q->r[j + 7 * i] = Q_temp.r[j + 4 * i];
        }
    }
}

void EKF_predict(EKF *ekf, Vec *gyro, float dt) {
    Q_prediction(ekf, dt);
    predict_state(ekf, gyro, dt);
    F_jacobian(ekf, gyro, dt);
    mat_transpose(ekf->F, ekf->F_trans);
    mat_mult(ekf->F, ekf->P, ekf->temp_mat1);
    mat_mult(ekf->temp_mat1, ekf->F_trans, ekf->temp_mat2);
    mat_add(ekf->temp_mat2, ekf->Q, ekf->P);
}


VEC_ALLOC_STATIC(tmp_x, 4);
void h_prediction(EKF *ekf) {
    tmp_x.r[0] = ekf->x->r[0];
    tmp_x.r[1] = ekf->x->r[1];
    tmp_x.r[2] = ekf->x->r[2];
    tmp_x.r[3] = ekf->x->r[3];
    vec_normalize(&tmp_x);
    quat_from_vec4(&tmp_x, ekf->q);
    quat_to_rotation_matrix(ekf->q, ekf->rot);
    mat_transpose(ekf->rot, ekf->rot_inv);

    mat_vec_mult(ekf->rot_inv,ekf->acc_refrence, ekf->vtmp);
    ekf->h->r[0] = ekf->vtmp->r[0];
    ekf->h->r[1] = ekf->vtmp->r[1];
    ekf->h->r[2] = ekf->vtmp->r[2];

    mat_vec_mult(ekf->rot_inv,ekf->mag_refrence, ekf->vtmp);
    ekf->h->r[3] = ekf->vtmp->r[0];
    ekf->h->r[4] = ekf->vtmp->r[1];
    ekf->h->r[5] = ekf->vtmp->r[2];
}

void H_jacobian(EKF *ekf) {
    float w = ekf->x->r[0];
    float i = ekf->x->r[1];
    float j = ekf->x->r[2];
    float k = ekf->x->r[3];

    float x = ekf->acc_refrence->r[0];
    float y = ekf->acc_refrence->r[1];
    float z = ekf->acc_refrence->r[2];

    ekf->H->r[0 + 7 * 0] = 2 * (x * w + y * k - z * j);
    ekf->H->r[1 + 7 * 0] = 2 * (x * i + y * j + z * k);
    ekf->H->r[2 + 7 * 0] = 2 * (- x * j + y * i - z * w);
    ekf->H->r[3 + 7 * 0] = 2 * (- x * k + y * w + z * i);

    ekf->H->r[0 + 7 * 1] = 2 * (- x * k + y * w + z * i);
    ekf->H->r[1 + 7 * 1] = 2 * (x * j - y * i + z * w);
    ekf->H->r[2 + 7 * 1] = 2 * (x * i + y * j + z * k);
    ekf->H->r[3 + 7 * 1] = 2 * (- x * w - y * k + z * j);

    ekf->H->r[0 + 7 * 2] = 2 * (x * j - y * i + z * w);
    ekf->H->r[1 + 7 * 2] = 2 * (x * k - y * w - z * i);
    ekf->H->r[2 + 7 * 2] = 2 * (x * w + y * k - z * j);
    ekf->H->r[3 + 7 * 2] = 2 * (x * i + y * j + z * k);

    x = ekf->mag_refrence->r[0];
    y = ekf->mag_refrence->r[1];
    z = ekf->mag_refrence->r[2];

    ekf->H->r[0 + 7 * 3] = 2 * (x * w + y * k - z * j);
    ekf->H->r[1 + 7 * 3] = 2 * (x * i + y * j + z * k);
    ekf->H->r[2 + 7 * 3] = 2 * (- x * j + y * i - z * w);
    ekf->H->r[3 + 7 * 3] = 2 * (- x * k + y * w + z * i);

    ekf->H->r[0 + 7 * 4] = 2 * (- x * k + y * w + z * i);
    ekf->H->r[1 + 7 * 4] = 2 * (x * j - y * i + z * w);
    ekf->H->r[2 + 7 * 4] = 2 * (x * i + y * j + z * k);
    ekf->H->r[3 + 7 * 4] = 2 * (- x * w - y * k + z * j);

    ekf->H->r[0 + 7 * 5] = 2 * (x * j - y * i + z * w);
    ekf->H->r[1 + 7 * 5] = 2 * (x * k - y * w - z * i);
    ekf->H->r[2 + 7 * 5] = 2 * (x * w + y * k - z * j);
    ekf->H->r[3 + 7 * 5] = 2 * (x * i + y * j + z * k);
}

void EKF_update(EKF *ekf) {
    h_prediction(ekf);
    vec_sub(ekf->z, ekf->h, ekf->y);
    H_jacobian(ekf);

    mat_transpose(ekf->H, ekf->H_trans);
    mat_mult(ekf->P, ekf->H_trans, ekf->tmp1);
    mat_mult(ekf->H, ekf->tmp1, ekf->tmp2);
    mat_add(ekf->tmp2, ekf->R, ekf->S);

    // this->K = this->P * this->H.transpose() * S.inverse();
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

    mat_copy(ekf->tmp3, ekf->P);

    ekf->attitude->q = ekf->q->q;
    ekf->attitude->i = ekf->q->i;
    ekf->attitude->j = ekf->q->j;
    ekf->attitude->k = ekf->q->k;
    ekf->bias->r[0] = ekf->x->r[4];
    ekf->bias->r[1] = ekf->x->r[5];
    ekf->bias->r[2] = ekf->x->r[6];
}

VEC_ALLOC_STATIC(gyro_mean, 3);
VEC_ALLOC_STATIC(acc_mean, 3);
VEC_ALLOC_STATIC(mag_mean, 3);

VEC_ALLOC_STATIC(M_gyro, 3);
VEC_ALLOC_STATIC(M_acc, 3);
VEC_ALLOC_STATIC(M_mag, 3);
VEC_ALLOC_STATIC(temp, 3);
VEC_ALLOC_STATIC(temp2, 3);

int cnt = 0;
void EKF_init_incremental(EKF *ekf, Vec *gyro, Vec *acc, Vec *mag) {
    cnt += 1;

    vec_copy(&gyro_mean, &temp2);
    vec_sub(gyro, &gyro_mean, &temp);
    vec_scalar_mult(&temp, 1.0/cnt);
    vec_add(&gyro_mean, &temp, &gyro_mean);

    vec_sub(gyro, &temp2, &temp2);
    vec_sub(gyro, &gyro_mean, &temp);
    vec_mult(&temp2, &temp, &temp);
    vec_add(&M_gyro, &temp, &M_gyro);


    vec_copy(&acc_mean, &temp2);
    vec_sub(acc, &acc_mean, &temp);
    vec_scalar_mult(&temp, 1.0/cnt);
    vec_add(&acc_mean, &temp, &acc_mean);

    vec_sub(acc, &temp2, &temp2);
    vec_sub(acc, &acc_mean, &temp);
    vec_mult(&temp2, &temp, &temp);
    vec_add(&M_acc, &temp, &M_acc);


    vec_copy(&mag_mean, &temp2);
    vec_sub(mag, &mag_mean, &temp);
    vec_scalar_mult(&temp, 1.0/cnt);
    vec_add(&mag_mean, &temp, &mag_mean);

    vec_sub(mag, &temp2, &temp2);
    vec_sub(mag, &mag_mean, &temp);
    vec_mult(&temp2, &temp, &temp);
    vec_add(&M_mag, &temp, &M_mag);
    //vec_print(&M_gyro);
    //vec_print(&M_mag);
    //vec_print(&M_acc);
}
MAT_ALLOC_STATIC(tmp1, 4, 3);
MAT_ALLOC_STATIC(Q_q, 4, 4);
MAT_ALLOC_STATIC(Q_g, 3, 3);

void EKF_init_final(EKF *ekf) {
    vec_scalar_mult(&M_gyro, 1.0 / (cnt - 1));
    vec_scalar_mult(&M_acc, 1.0 / (cnt - 1));
    vec_scalar_mult(&M_mag, 1.0 / (cnt - 1));

    float ss_gyro = vec_norm(&M_gyro);
    float ss_acc = vec_norm(&M_acc);
    float ss_mag = vec_norm(&M_mag);

    float m_a_c_x = acc_mean.r[0];
    float m_a_c_y = acc_mean.r[1];
    float m_a_c_z = acc_mean.r[2];

    float m_x = mag_mean.r[0];
    float m_y = mag_mean.r[1];
    float m_z = mag_mean.r[2];

    float roll = atan2f(m_a_c_y, m_a_c_z);
    float pitch = atanf(- m_a_c_x / sqrtf(m_a_c_y * m_a_c_y + m_a_c_z * m_a_c_z));

    float mm_c_x = m_x * cosf(pitch) + m_z * sinf(pitch);
    float mm_c_y = m_x * sinf(roll) * sinf(pitch) + m_y * cosf(roll) - m_z * sinf(roll) * cosf(pitch);

    float yaw = atan2f(- mm_c_y, mm_c_x);

    quat_from_rpy(ekf->attitude, roll, pitch, yaw);
    ekf->x->r[0] = ekf->attitude->q;
    ekf->x->r[1] = ekf->attitude->i; ekf->x->r[2] = ekf->attitude->j; ekf->x->r[3] = ekf->attitude->k;
    ekf->x->r[4] = 0.0;
    ekf->x->r[5] = 0.0;
    ekf->x->r[6] = 0.0;

    //ss_acc = 0.01 * 0.01;
    //ss_mag = 0.02 * 0.02;
    //ss_gyro = 0.3 * 0.3;
    ekf->R->r[0 + 6 * 0] = ss_acc * ss_acc;
    ekf->R->r[1 + 6 * 1] = ss_acc * ss_acc;
    ekf->R->r[2 + 6 * 2] = ss_acc * ss_acc;
    ekf->R->r[3 + 6 * 3] = ss_mag * ss_mag;
    ekf->R->r[4 + 6 * 4] = ss_mag * ss_mag;
    ekf->R->r[5 + 6 * 5] = ss_mag * ss_mag;

    ekf->gyro_variance = ss_gyro * ss_gyro;

    ekf->acc_refrence->r[0] = 0;
    ekf->acc_refrence->r[1] = 0;
    ekf->acc_refrence->r[2] = 1;

    quat_to_rotation_matrix(ekf->attitude, ekf->rot);
    mat_vec_mult(ekf->rot, &mag_mean, ekf->mag_refrence);

    vec_normalize(ekf->mag_refrence);
    mat_diag(ekf->P, 1);
}
uint64_t next_mag_time = 0;
void update_measurements() {
    if (next_mag_time <= now()) {
        EKF_update_acc(&ekf, &LSM9DS1_acc_filtered);
        EKF_update_mag(&ekf, &LSM9DS1_mag_filtered, &LSM9DS1_acc_filtered);
        next_mag_time = now() + 10 * MILLISECONDS;
    }
}


VEC_ALLOC_STATIC(acc_, 3);
VEC_ALLOC_STATIC(gyro_, 3);
VEC_ALLOC_STATIC(mag_, 3);
volatile void attitude_thread() {
    sleep(20 * MILLISECONDS);
    for (int i = 0; i < CALIB_COUNT; i++) {
        EKF_init_incremental(&ekf, &LSM9DS1_gyro_filtered, &LSM9DS1_acc_filtered, &LSM9DS1_mag_filtered);
        sleep(5* MILLISECONDS);
    }

    EKF_init_final(&ekf);

    uint64_t next_time = now();
    while(1) {
        next_time = now() + 5 * MILLISECONDS;
        update_measurements();
        EKF_predict(&ekf, &LSM9DS1_gyro_filtered, 0.005);
        EKF_update(&ekf);
        scheduler_disable();
        os_printf("Attitude: ");
        quat_print(ekf.attitude);
        scheduler_enable();
        sleep_until(next_time);
    }
}
