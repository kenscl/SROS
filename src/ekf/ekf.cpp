#include "ekf.h"
#include <cmath>


EKF::EKF() {}

void EKF::init(Vec3 *gyro, Vec3 *acc, Vec3 *mag) {

    int num_init = 100;
    Vec3 mean_gyro;
    Vec3 mean_acc;
    Vec3 mean_mag;
    for (int i = 0; i < num_init; ++i) {
        mean_gyro = mean_gyro + gyro[i] / num_init;
        mean_acc = mean_acc - acc[i] / num_init;
        mean_mag = mean_mag + mag[i] / num_init;
    }

    float roll_init = atan2(mean_acc[1], mean_acc[2]);
    float pitch_init = atan2(mean_acc[0], sqrt(mean_acc[1]*mean_acc[1]+mean_acc[2]*mean_acc[2]));

    float cp = cos(pitch_init);
    float sp = sin(pitch_init);
    float cr = cos(roll_init);
    float sr = sin(roll_init);

    this->Rot[0][0] = cp;
    this->Rot[0][1] = sp * sr;
    this->Rot[0][2] = sp * cr;

    this->Rot[1][0] = 0;
    this->Rot[1][1] = cr;
    this->Rot[1][2] = - sr;

    this->Rot[2][0] = -sp;
    this->Rot[2][1] = cp * sr;
    this->Rot[2][2] = cp * cr;

    Vec3 mr = this->Rot * mean_mag;
    float yaw_init = atan2(-mr[1], mr[0]);
    Quaternion q_init(roll_init, pitch_init, yaw_init);
    
    this->x[0] = q_init.q;
    this->x[1] = q_init.i;
    this->x[2] = q_init.j;
    this->x[3] = q_init.k;
    this->x[7] = mean_gyro[0];
    this->x[8] = mean_gyro[1];
    this->x[9] = mean_gyro[2];

    this->P = Mat<10,10>().identity() * 10;

    float v_bias = 1.0e-11;

    Vec3 gyro_sum;
    Vec3 acc_sum;
    float mag_sum;

    for (int i = 0; i < num_init; ++i) {
        gyro_sum = gyro_sum + (gyro[i] - mean_gyro).mult(gyro[i] - mean_gyro);
        acc_sum = acc_sum + (acc[i] + mean_acc).mult(acc[i] + mean_acc);

        Vec3 mw = this->Rot * mag[i]; 
        float mag_yaw = atan2(-mw[1], mw[0]);
        mag_sum = mag_sum + (mag_yaw - yaw_init) * (mag_yaw - yaw_init);
    }

    float s_gyro = (sqrt(gyro_sum[0]/(num_init-1)) + sqrt(gyro_sum[1]/(num_init-1)) + sqrt(gyro_sum[2]/(num_init-1))) / 3;
    float s_acc = (sqrt(acc_sum[0]/(num_init-1)) + sqrt(acc_sum[1]/(num_init-1)) + sqrt(acc_sum[2]/(num_init-1))) / 3;
    float s_yaw = sqrt(mag_sum/(num_init-1));


    this->R[0][0] = 2.12425429e-06;
    this->R[1][1] = 2.12425429e-06;
    this->R[2][2] = 2.12425429e-06;
    this->R[3][3] = 7.98243297e-05;


    Mat<10,6> Fu;
    Fu[4][0] = 1;
    Fu[5][1] = 1;
    Fu[6][2] = 1;
    Fu[7][3] = 1;
    Fu[8][4] = 1;
    Fu[9][5] = 1;


    float q0 = this->x[0];
    float q1 = this->x[1]; 
    float q2 = this->x[2];
    float q3 = this->x[3];

    this->Rot[0][0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    this->Rot[0][1] = 2 * (q1 * q2 - q0 * q3);
    this->Rot[0][2] = 2 * (q0 * q2 + q1 * q3);

    this->Rot[1][0] = 2 * (q1 * q2 + q0 * q3);
    this->Rot[1][1] = (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3);
    this->Rot[1][2] = 2 * (q2 * q3 - q0 * q1);

    this->Rot[2][0] = 2 * (q1 * q3 - q0 * q2);
    this->Rot[2][1] = 2 * (q0 * q1 + q2 * q3);
    this->Rot[2][2] = (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    this->Rot_inv = Rot.transpose();

    Mat<6,6> U;
    U[0][0] = 3.47930986e-04;
    U[1][1] = 3.47930986e-04;
    U[2][2] = 3.47930986e-04;
    U[3][3] = v_bias;
    U[4][4] = v_bias;
    U[5][5] = v_bias;
    this->Q = Fu * U * Fu.transpose();
}

void EKF::update_acc(Vec3 acc) {
    this->y[0] = acc[0];
    this->y[1] = acc[1];
    this->y[2] = acc[2];
}

void EKF::update_mag(Vec3 mag) {
    Vec3 m = this->Rot * mag;
    m[2] = 0;
    m =  this->Rot_inv * m;
    float yaw = atan2(- m[1], m[0]);
    this->y[3] = yaw;
}

void EKF::predict(Vec3 gyro, float dt) {
    float q0 = this->x[0];
    float q1 = this->x[1]; 
    float q2 = this->x[2];
    float q3 = this->x[3];
    float wx = this->x[4];
    float wy = this->x[5];
    float wz = this->x[6];
    float xgx = this->x[7];
    float xgy = this->x[8];
    float xgz = this->x[9];
    Vec<10> x_ = this->x;

    Quaternion q(this->x[0], this->x[1], this->x[2], this->x[3]);
    Quaternion w(0, wx, wy, wz);
    q = q + q * w * 0.5 * dt;
    q = q.normalize();
    this->x[0] = q.q;
    this->x[1] = q.i;
    this->x[2] = q.j;
    this->x[3] = q.k;
    this->x[4] = gyro[0] - xgx;
    this->x[5] = gyro[1] - xgy;
    this->x[6] = gyro[2] - xgz;
    this->x[7] = xgx;
    this->x[8] = xgy;
    this->x[9] = xgz;

    this->F[0][0] = 1;
    this->F[0][1] = -0.5 * dt * wx;
    this->F[0][2] = -0.5 * dt * wy;
    this->F[0][3] = -0.5 * dt * wz;
    this->F[0][4] = -0.5 * dt * q1;
    this->F[0][5] = -0.5 * dt * q2;
    this->F[0][6] = -0.5 * dt * q3;
    this->F[0][7] = 0;
    this->F[0][8] = 0;
    this->F[0][9] = 0;

    this->F[1][0] = 0.5 * dt * wx;
    this->F[1][1] = 1;
    this->F[1][2] = 0.5 * dt * wz;
    this->F[1][3] = -0.5 * dt * wy;
    this->F[1][4] = 0.5 * dt * q0;
    this->F[1][5] = -0.5 * dt * q3;
    this->F[1][6] = 0.5 * dt * q2;
    this->F[1][7] = 0;
    this->F[1][8] = 0;
    this->F[1][9] = 0;

    this->F[2][0] = 0.5 * dt * wy;
    this->F[2][1] = -0.5 * dt * wz;
    this->F[2][2] = 1;
    this->F[2][3] = 0.5 * dt * wx;
    this->F[2][4] = 0.5 * dt * q3;
    this->F[2][5] = 0.5 * dt * q0;
    this->F[2][6] = -0.5 * dt * q1;
    this->F[2][7] = 0;
    this->F[2][8] = 0;
    this->F[2][9] = 0;

    this->F[3][0] = 0.5 * dt * wz;
    this->F[3][1] = 0.5 * dt * wy;
    this->F[3][2] = -0.5 * dt * wx;
    this->F[3][3] = 1;
    this->F[3][4] = -0.5 * dt * q2;
    this->F[3][5] = 0.5 * dt * q1;
    this->F[3][6] = 0.5 * dt * q0;
    this->F[3][7] = 0;
    this->F[3][8] = 0;
    this->F[3][9] = 0;

    this->F[4][0] = 0;
    this->F[4][1] = 0;
    this->F[4][2] = 0;
    this->F[4][3] = 0;
    this->F[4][4] = 0;
    this->F[4][5] = 0;
    this->F[4][6] = 0;
    this->F[4][7] = -1;
    this->F[4][8] = 0;
    this->F[4][9] = 0;

    this->F[5][0] = 0;
    this->F[5][1] = 0;
    this->F[5][2] = 0;
    this->F[5][3] = 0;
    this->F[5][4] = 0;
    this->F[5][5] = 0;
    this->F[5][6] = 0;
    this->F[5][7] = 0;
    this->F[5][8] = -1;
    this->F[5][9] = 0;

    this->F[6][0] = 0;
    this->F[6][1] = 0;
    this->F[6][2] = 0;
    this->F[6][3] = 0;
    this->F[6][4] = 0;
    this->F[6][5] = 0;
    this->F[6][6] = 0;
    this->F[6][7] = 0;
    this->F[6][8] = 0;
    this->F[6][9] = -1;

    this->F[7][0] = 0;
    this->F[7][1] = 0;
    this->F[7][2] = 0;
    this->F[7][3] = 0;
    this->F[7][4] = 0;
    this->F[7][5] = 0;
    this->F[7][6] = 0;
    this->F[7][7] = 1;
    this->F[7][8] = 0;
    this->F[7][9] = 0;

    this->F[8][0] = 0;
    this->F[8][1] = 0;
    this->F[8][2] = 0;
    this->F[8][3] = 0;
    this->F[8][4] = 0;
    this->F[8][5] = 0;
    this->F[8][6] = 0;
    this->F[8][7] = 0;
    this->F[8][8] = 1;
    this->F[8][9] = 0;

    this->F[9][0] = 0;
    this->F[9][1] = 0;
    this->F[9][2] = 0;
    this->F[9][3] = 0;
    this->F[9][4] = 0;
    this->F[9][5] = 0;
    this->F[9][6] = 0;
    this->F[9][7] = 0;
    this->F[9][8] = 0;
    this->F[9][9] = 1;

    this->P = this->F * this->P * this->F.transpose() + this->Q;

    q0 = this->x[0];
    q1 = this->x[1];
    q2 = this->x[2];
    q3 = this->x[3];

    this->Rot[0][0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    this->Rot[0][1] = 2 * (q1 * q2 - q0 * q3);
    this->Rot[0][2] = 2 * (q0 * q2 + q1 * q3);

    this->Rot[1][0] = 2 * (q1 * q2 + q0 * q3);
    this->Rot[1][1] = (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3);
    this->Rot[1][2] = 2 * (q2 * q3 - q0 * q1);

    this->Rot[2][0] = 2 * (q1 * q3 - q0 * q2);
    this->Rot[2][1] = 2 * (q0 * q1 + q2 * q3);
    this->Rot[2][2] = (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    this->Rot_inv = Rot.transpose();

    q = Quaternion(this->x[0], this->x[1], this->x[2], this->x[3]);


    Vec3 rev_g;
    rev_g[2] = -1;
    Vec3 z_acc = Rot_inv * rev_g;
    float zyaw = q.to_rpy()[2];
    this->z[0] = z_acc [0];
    this->z[1] = z_acc [1];
    this->z[2] = z_acc [2];
    this->z[3] = zyaw;

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

    this->H[0][0] =  2 * q2;
    this->H[0][1] = -2 * q3;
    this->H[0][2] = 2 * q0;
    this->H[0][3] = -2 * q1;
    this->H[0][4] = 0;
    this->H[0][5] = 0;
    this->H[0][6] = 0;
    this->H[0][7] = 0;
    this->H[0][8] = 0;
    this->H[0][9] = 0;

    this->H[1][0] = -2 * q1;
    this->H[1][1] = -2 * q0;
    this->H[1][2] = -2 * q3;
    this->H[1][3] = -2 * q2;
    this->H[1][4] = 0;
    this->H[1][5] = 0;
    this->H[1][6] = 0;
    this->H[1][7] = 0;
    this->H[1][8] = 0;
    this->H[1][9] = 0;

    this->H[2][0] = -2 * q0;
    this->H[2][1] = 2 * q1;
    this->H[2][2] = 2 * q2;
    this->H[2][3] = -2 * q3;
    this->H[2][4] = 0;
    this->H[2][5] = 0;
    this->H[2][6] = 0;
    this->H[2][7] = 0;
    this->H[2][8] = 0;
    this->H[2][9] = 0;

    this->H[3][0] = dhm_dqw;
    this->H[3][1] = dhm_dqi;
    this->H[3][2] = dhm_dqj;
    this->H[3][3] = dhm_dqk;
    this->H[3][4] = 0;
    this->H[3][5] = 0;
    this->H[3][6] = 0;
    this->H[3][7] = 0;
    this->H[3][8] = 0;
    this->H[3][9] = 0;
}

void EKF::update() {
    Vec4 v = this->y - this->z;
    if (v[3] > M_PI) {
      v[3] -= 2 * M_PI;
    } else if (v[3] < -M_PI) {
        v[3] += 2 * M_PI; 
    }

    Mat<4,4> S = this->H * this->P * this->H.transpose() + this->R;
    this->K = this->P * this->H.transpose() * S.inverse();
    this->x = this->x + this->K * v;
    Quaternion q(this->x[0], this->x[1], this->x[2], this->x[3]);
    q = q.normalize();
    this->x[0] = q.q;
    this->x[1] = q.i;
    this->x[2] = q.j;
    this->x[3] = q.k;
    this->P = (Mat<10,10>().identity() - (this->K * this->H)) * this->P;
    this->attitude = q;
}

int gyro_cnt = 0;
int acc_cnt = 0;
int mag_cnt = 0;
EKF ekf;
Vec3 gyro[100];
Vec3 mag[100];
Vec3 acc[100];
int has_init = 0;
volatile void attitude_thread() {
    uint64_t next_mag_time = now();
    uint32_t last_time = now_high_accuracy();
    while (1) {
        volatile uint64_t next_time = now() + 3 * MILLISECONDS;
        if (next_mag_time < now()) { // we only get this every 50 milliseconds, so setting the value more ofter just wastes computation
            next_mag_time = now() + 50 * MILLISECONDS;
            ekf.update_mag(LSM9DS1_mag);

            if (mag_cnt < 100) {
                mag[mag_cnt] = LSM9DS1_mag;
                mag_cnt++;
            }
        }

        ekf.update_acc(LSM9DS1_acc);
        if (has_init) {
            ekf.predict(LSM9DS1_gyro, (float) (now_high_accuracy() - last_time) / 1e6);
            os_printf("dt: %f\n", (float) (now_high_accuracy() - last_time) / 1e6);
            last_time = now_high_accuracy();
            ekf.update();
        }

        if (acc_cnt < 100) {
            acc[acc_cnt] = LSM9DS1_acc;
            acc_cnt++;
        }
        if (gyro_cnt < 100) {
            gyro[gyro_cnt] = LSM9DS1_gyro;
            gyro_cnt++;
        }
        if (gyro_cnt > 100 && mag_cnt > 100 && acc_cnt > 100 && !has_init) {
            ekf.init(gyro, acc, mag);
            has_init = 1;
        }
        sleep_until(next_time);
    }
}
