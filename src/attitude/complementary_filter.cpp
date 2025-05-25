#include "complementary_filter.h"

void Complementary_Filter::init(Vec3 *gyro, Vec3 *acc, Vec3 *mag) {
  int num_init = 100;
  Vec3 mean_gyro;
  Vec3 mean_acc;
  Vec3 mean_mag;
  for (int i = 0; i < num_init; ++i) {
    mean_gyro = mean_gyro + gyro[i] / num_init;
    mean_acc = mean_acc + acc[i] / num_init;
    mean_mag = mean_mag + mag[i] / num_init;
  }

  this->update_mag(mean_mag);
  this->update_acc(mean_acc);

  float roll_init = atan2(mean_acc[1], -mean_acc[2]);
  float pitch_init = atan(-mean_acc[0] / sqrt(mean_acc[1] * mean_acc[1] + mean_acc[2] * mean_acc[2]));

  float mean_mag_x_comp = mean_mag[0] * cos(pitch_init) + mean_mag[2] * sin(pitch_init);
  float mean_mag_y_comp = mean_mag[0] * sin(roll_init) * sin(pitch_init) + mean_mag[1] * cos(roll_init) -
                     mean_mag[2] * sin(roll_init) * cos(pitch_init);

  float yaw_init = atan2(mean_mag_y_comp, -mean_mag_x_comp);
  if (yaw_init > M_PI)
    yaw_init -= 2.0 * M_PI;
  else if (yaw_init < -M_PI)
    yaw_init += 2.0 * M_PI;

  this->attitude = Quaternion (roll_init, pitch_init, yaw_init);
  this->beta = 0.1;
}

void Complementary_Filter::update_acc(Vec3 acc) {
  this->roll = atan2(acc[1], acc[2]);
  this->pitch = atan(-acc[0] / sqrt(acc[1] * acc[1] + acc[2] * acc[2]));
  this->prediction_am[0] = this->roll;
  this->prediction_am[1] = this->pitch;
}

void Complementary_Filter::update_mag(Vec3 mag) {
  float mag_x_comp = mag[0] * cos(this->pitch) + mag[1] * sin(this->roll) * sin(this->pitch) + mag[2] * sin(this->pitch) * cos(this->roll);
  float mag_y_comp = mag[1] * cos(this->roll) - mag[2] * sin(this->roll);

  this->yaw = atan2(mag_y_comp, -mag_x_comp);
  if (this->yaw > M_PI)
    this->yaw -= 2.0 * M_PI;
  else if (this->yaw < -M_PI)
    this->yaw += 2.0 * M_PI;

  this->prediction_am[2] = this->yaw;
}

void Complementary_Filter::predict(Vec3 gyro, float dt) {
  Vec3 rpy = this->attitude.to_rpy();
  this->attitude = Quaternion((rpy + gyro * dt) * (1 - this->beta) + this->prediction_am * beta);
}

Complementary_Filter cf;
int gyro_cnt_cf = 0;
int acc_cnt_cf = 0;
int mag_cnt_cf = 0;
Vec3 gyro_cf[100];
Vec3 mag_cf[100];
Vec3 acc_cf[100];
int has_init_cf = 0;

volatile void attitude_thread_complementary_filter() {
  uint64_t next_mag_time_cf = now();
  uint32_t last_time = now();
  while (1) {
    volatile uint64_t next_time = now() + 3 * MILLISECONDS;

    if (has_init_cf) {

      // mag update
      if (next_mag_time_cf < now()) { // we only get this every n milliseconds, so
                                   // setting the value more often is
        next_mag_time_cf = now() + 15 * MILLISECONDS;
        cf.update_mag(LSM9DS1_mag_filtered.normalize());

        os_printf("Attitude, ");
        cf.attitude.print_bare();
      }

      // acc update
      cf.update_acc(LSM9DS1_acc_filtered);
      float dt = (float)(now() - last_time) / SECONDS;
      last_time = now();

      // state prediction
      cf.predict(LSM9DS1_gyro_filtered / 180 * M_PI, dt);
    }

    // init
    if (acc_cnt_cf < 100) {
      acc_cf[acc_cnt_cf] = LSM9DS1_acc_filtered.normalize();
      acc_cnt_cf++;
    }

    if (gyro_cnt_cf < 100) {
      gyro_cf[gyro_cnt_cf] = LSM9DS1_gyro_filtered;
      gyro_cnt_cf++;
    }

    if (next_mag_time_cf < now() && mag_cnt_cf < 100) {
      next_mag_time_cf = now() + 15 * MILLISECONDS;
      mag_cf[mag_cnt_cf] = LSM9DS1_mag_filtered.normalize();
      mag_cnt_cf++;
    }

    if (gyro_cnt_cf == 100 && mag_cnt_cf == 100 && acc_cnt_cf == 100 && !has_init_cf) {
      cf.init(gyro_cf, acc_cf, mag_cf);
      has_init_cf = 1;
    }

    sleep_until(next_time);
  }
}
