#include "ekf.h"
#include "../config.h"
#include "../math/matrix.h"
#include "../math/quaternion.h"
#include <math.h>
#include <stdint.h>

EKF ekf;

void EKF_update_acc(EKF *ekf, Vec3 *acc)
{
    if (fabsf(1 - acc->norm()) < 0.2)
    {
        Vec3 norm = acc->normalize();
        ekf->z[0] = norm[0];
        ekf->z[1] = norm[1];
        ekf->z[2] = norm[2];
    }
}

void EKF_update_mag(EKF *ekf, Vec3 *mag, Vec3 *acc)
{
    Vec3 norm = mag->normalize();
    ekf->z[3] = norm[0];
    ekf->z[4] = norm[1];
    ekf->z[5] = norm[2];
}

void predict_state(EKF *ekf, Vec3 *gyro, float dt)
{
    float qw = ekf->x[0];
    float qi = ekf->x[1];
    float qj = ekf->x[2];
    float qk = ekf->x[3];

    float wx = (*gyro)[0]; // - ekf->bias->r[0];
    float wy = (*gyro)[1]; // - ekf->bias->r[1];
    float wz = (*gyro)[2]; // - ekf->bias->r[2];

    float qwp1 = qw + 0.5f * dt * (-wx * qi - wy * qj - wz * qk);
    float qip1 = qi + 0.5f * dt * (wx * qw - wy * qk + wz * qj);
    float qjp1 = qj + 0.5f * dt * (wx * qk + wy * qw - wz * qi);
    float qkp1 = qk + 0.5f * dt * (-wx * qj + wy * qi + wz * qw);

    float norm = 1; // sqrtf(qwp1*qwp1 + qip1*qip1 + qjp1*qjp1 + qkp1*qkp1);
    qwp1 /= norm;
    qip1 /= norm;
    qjp1 /= norm;
    qkp1 /= norm;

    ekf->x[0] = qwp1;
    ekf->x[1] = qip1;
    ekf->x[2] = qjp1;
    ekf->x[3] = qkp1;
    ekf->x = ekf->x.normalize();
}

void F_jacobian(EKF *ekf, Vec3 *gyro, float dt)
{
    ekf->bias[0] = 0;
    ekf->bias[1] = 0;
    ekf->bias[2] = 0;
    ekf->x[4] = 0;
    ekf->x[5] = 0;
    ekf->x[6] = 0;
    float wx = (*gyro)[0] - ekf->bias[0];
    float wy = (*gyro)[1] - ekf->bias[1];
    float wz = (*gyro)[2] - ekf->bias[2];

    ekf->F[0][0] = 1;
    ekf->F[0][1] = -0.5 * dt * wx;
    ekf->F[0][2] = -0.5 * dt * wy;
    ekf->F[0][3] = -0.5 * dt * wz;

    ekf->F[1][0] = 0.5 * dt * wx;
    ekf->F[1][1] = 1;
    ekf->F[1][2] = 0.5 * dt * wz;
    ekf->F[1][3] = -0.5 * dt * wy;

    ekf->F[2][0] = 0.5 * dt * wy;
    ekf->F[2][1] = -0.5 * dt * wz;
    ekf->F[2][2] = 1;
    ekf->F[2][3] = 0.5 * dt * wx;

    ekf->F[3][0] = 0.5 * dt * wz;
    ekf->F[3][1] = 0.5 * dt * wy;
    ekf->F[3][2] = -0.5 * dt * wx;
    ekf->F[3][3] = 1;

    float qw = ekf->x[0];
    float qi = ekf->x[1];
    float qj = ekf->x[2];
    float qk = ekf->x[3];
}

void Q_prediction(EKF *ekf, float dt)
{
    float w = ekf->attitude.q;
    float x = ekf->attitude.i;
    float y = ekf->attitude.j;
    float z = ekf->attitude.k;
    ekf->W[0][0] = -x * dt * 0.5;
    ekf->W[0][1] = -y * dt * 0.5;
    ekf->W[0][2] = -z * dt * 0.5;

    ekf->W[1][0] = w * dt * 0.5;
    ekf->W[1][1] = -z * dt * 0.5;
    ekf->W[1][2] = y * dt * 0.5;

    ekf->W[2][0] = z * dt * 0.5;
    ekf->W[2][1] = w * dt * 0.5;
    ekf->W[2][2] = -x * dt * 0.5;

    ekf->W[3][0] = -y * dt * 0.5;
    ekf->W[3][1] = x * dt * 0.5;
    ekf->W[3][2] = w * dt * 0.5;

    ekf->W_trans = ekf->W.transpose();
    ekf->Q = ekf->W * ekf->W_trans * ekf->gyro_variance;
}

void EKF_predict(EKF *ekf, Vec3 *gyro, float dt)
{
    Q_prediction(ekf, dt);
    predict_state(ekf, gyro, dt);
    F_jacobian(ekf, gyro, dt);

    ekf->F_trans = ekf->F.transpose();
    ekf->P = (ekf->F * ekf->P) * ekf->F_trans + ekf->Q;
}

void h_prediction(EKF *ekf)
{
    Vec4 tmp_x;
    tmp_x[0] = ekf->x[0];
    tmp_x[1] = ekf->x[1];
    tmp_x[2] = ekf->x[2];
    tmp_x[3] = ekf->x[3];
    tmp_x = tmp_x.normalize();
    ekf->q = Quat(tmp_x);
    ekf->rot = ekf->q.to_rotation_matrix();
    ekf->rot_inv = ekf->rot.transpose();

    Vec3 vtmp = ekf->rot_inv * ekf->acc_refrence;
    ekf->h[0] = vtmp[0];
    ekf->h[1] = vtmp[1];
    ekf->h[2] = vtmp[2];

    vtmp = ekf->rot_inv * ekf->mag_refrence;
    ekf->h[3] = vtmp[0];
    ekf->h[4] = vtmp[1];
    ekf->h[5] = vtmp[2];
}

void H_jacobian(EKF *ekf)
{
    float w = ekf->x[0];
    float i = ekf->x[1];
    float j = ekf->x[2];
    float k = ekf->x[3];

    float x = ekf->acc_refrence[0];
    float y = ekf->acc_refrence[1];
    float z = ekf->acc_refrence[2];

    ekf->H[0][0] = 2 * (x * w + y * k - z * j);
    ekf->H[0][1] = 2 * (x * i + y * j + z * k);
    ekf->H[0][2] = 2 * (-x * j + y * i - z * w);
    ekf->H[0][3] = 2 * (-x * k + y * w + z * i);

    ekf->H[1][0] = 2 * (-x * k + y * w + z * i);
    ekf->H[1][1] = 2 * (x * j - y * i + z * w);
    ekf->H[1][2] = 2 * (x * i + y * j + z * k);
    ekf->H[1][3] = 2 * (-x * w - y * k + z * j);

    ekf->H[2][0] = 2 * (x * j - y * i + z * w);
    ekf->H[2][1] = 2 * (x * k - y * w - z * i);
    ekf->H[2][2] = 2 * (x * w + y * k - z * j);
    ekf->H[2][3] = 2 * (x * i + y * j + z * k);

    x = ekf->mag_refrence[0];
    y = ekf->mag_refrence[1];
    z = ekf->mag_refrence[2];

    ekf->H[3][0] = 2 * (x * w + y * k - z * j);
    ekf->H[3][1] = 2 * (x * i + y * j + z * k);
    ekf->H[3][2] = 2 * (-x * j + y * i - z * w);
    ekf->H[3][3] = 2 * (-x * k + y * w + z * i);

    ekf->H[4][0] = 2 * (-x * k + y * w + z * i);
    ekf->H[4][1] = 2 * (x * j - y * i + z * w);
    ekf->H[4][2] = 2 * (x * i + y * j + z * k);
    ekf->H[4][3] = 2 * (-x * w - y * k + z * j);

    ekf->H[5][0] = 2 * (x * j - y * i + z * w);
    ekf->H[5][1] = 2 * (x * k - y * w - z * i);
    ekf->H[5][2] = 2 * (x * w + y * k - z * j);
    ekf->H[5][3] = 2 * (x * i + y * j + z * k);
}

void EKF_update(EKF *ekf)
{
    h_prediction(ekf);
    ekf->y = ekf->z - ekf->h;
    H_jacobian(ekf);

    ekf->H_trans = ekf->H.transpose();
    //(ekf->H * ekf->P * ekf->H_trans + ekf->R).print();
    ekf->K = ekf->P * ekf->H_trans * (ekf->H * ekf->P * ekf->H_trans + ekf->R).inverse();

    ekf->x = ekf->x + ekf->K * ekf->y;

    ekf->q = Quat(ekf->x[0], ekf->x[1], ekf->x[2], ekf->x[3]);
    ekf->q = ekf->q.normalize();
    ekf->x[0] = ekf->q.q;
    ekf->x[1] = ekf->q.i;
    ekf->x[2] = ekf->q.j;
    ekf->x[3] = ekf->q.k;

    ekf->P = (Mat<4, 4>().identity() - ekf->K * ekf->H) * ekf->P;

    ekf->attitude = ekf->q;
}

Vec3 gyro_mean, gyro_mean_old;
float M_gyro;

Vec3 acc_mean, acc_mean_old;
float M_acc;

Vec3 mag_mean, mag_mean_old;
float M_mag;

int cnt = 0;

void EKF_init_incremental(EKF *ekf, Vec3 *gyro, Vec3 *acc, Vec3 *mag)
{
    cnt += 1;

    gyro_mean_old = gyro_mean;
    gyro_mean = gyro_mean + (*gyro - gyro_mean) / cnt;
    M_gyro = M_gyro + (*gyro - gyro_mean_old) * (*gyro - gyro_mean);

    acc_mean_old = acc_mean;
    acc_mean = acc_mean + (*acc - acc_mean) / cnt;
    M_acc = M_acc + (*acc - acc_mean_old) * (*acc - acc_mean);

    mag_mean_old = mag_mean;
    mag_mean = mag_mean + (*mag - mag_mean) / cnt;
    M_mag = M_mag + (*mag - mag_mean_old) * (*mag - mag_mean);
}

void EKF_init_final(EKF *ekf)
{
    float ss_gyro = M_gyro / (cnt - 1);
    float ss_acc = M_acc / (cnt - 1);
    float ss_mag = M_mag / (cnt - 1);

    float m_a_c_x = acc_mean[0];
    float m_a_c_y = acc_mean[1];
    float m_a_c_z = acc_mean[2];

    float m_x = mag_mean[0];
    float m_y = mag_mean[1];
    float m_z = mag_mean[2];

    float roll = atan2f(m_a_c_y, m_a_c_z);
    float pitch = atanf(-m_a_c_x / sqrtf(m_a_c_y * m_a_c_y + m_a_c_z * m_a_c_z));

    float mm_c_x = m_x * cosf(pitch) + m_z * sinf(pitch);
    float mm_c_y = m_x * sinf(roll) * sinf(pitch) + m_y * cosf(roll) - m_z * sinf(roll) * cosf(pitch);

    float yaw = atan2f(-mm_c_y, mm_c_x);

    ekf->attitude = Quat(roll, pitch, yaw);
    ekf->x[0] = ekf->attitude.q;
    ekf->x[1] = ekf->attitude.i;
    ekf->x[2] = ekf->attitude.j;
    ekf->x[3] = ekf->attitude.k;
    ekf->x[4] = 0.0;
    ekf->x[5] = 0.0;
    ekf->x[6] = 0.0;

    //ss_acc = 0.0001;
    //ss_mag = 0.0001;
    //ss_gyro = 0.0001;

    os_printf("[ekf] ss_acc: %f, ss_mag %f, ss_gyro %f \n", ss_acc, ss_mag, ss_gyro);
    ekf->R[0][0] = ss_acc;
    ekf->R[1][1] = ss_acc;
    ekf->R[2][2] = ss_acc;
    ekf->R[3][3] = ss_mag;
    ekf->R[4][4] = ss_mag;
    ekf->R[5][5] = ss_mag;

    ekf->gyro_variance = ss_gyro;

    ekf->acc_refrence[0] = 0;
    ekf->acc_refrence[1] = 0;
    ekf->acc_refrence[2] = 1;

    ekf->rot = ekf->attitude.to_rotation_matrix();
    ekf->mag_refrence = (ekf->rot * mag_mean).normalize();

    ekf->P = ekf->P.diag(1);
}
uint64_t next_mag_time = 0;
void update_measurements()
{
    EKF_update_acc(&ekf, &LSM9DS1_acc_filtered);
    EKF_update_mag(&ekf, &LSM9DS1_mag_filtered, &LSM9DS1_acc_filtered);
}

volatile void attitude_thread()
{
    sleep(20 * MILLISECONDS);
    for (int i = 0; i < CALIB_COUNT; i++)
    {
        EKF_init_incremental(&ekf, &LSM9DS1_gyro_filtered, &LSM9DS1_acc_filtered, &LSM9DS1_mag_filtered);
        sleep(20 * MILLISECONDS);
    }

    EKF_init_final(&ekf);

    update_measurements();
    uint32_t next_time = now();
    uint32_t last_time_prediction = now();
    uint32_t last_time_update = now();
    uint32_t last_time_print = now();
    while (1)
    {
        if (last_time_prediction + 2 < now())
        {
            Vec3 zero;
            // EKF_predict(&ekf, &zero, 0.002);
            EKF_predict(&ekf, &LSM9DS1_gyro_filtered, 0.002);
            last_time_prediction = now();
        }
        if (last_time_update + 2 < now())
        {
            update_measurements();
            EKF_update(&ekf);
            last_time_update = now();
        }

#if PRINT_ATTITUDE == 1
        if (last_time_print + 100 < now())
        {
            last_time_print = now();
            os_printf("Attitude: ");
            ekf.attitude.print_bare();
        }
#endif

        yield();
    }
}
