//#include "complementary.h"
//float roll_acc = 0;
//float pitch_acc = 0;
//float yaw_mag = 0;
//float pitch =0;
//float roll =0;
//float yaw =0;
//
//float pitch_gyro =0;
//float roll_gyro =0;
//float yaw_gyro =0;
//
//void complementary_update(Vec *acc, Vec *mag) {
//    float ax = acc->r[0];
//    float ay = acc->r[1];
//    float az = acc->r[2];
//    float mx = mag->r[0];
//    float my = mag->r[1];
//    float mz = mag->r[2];
//    roll_acc = atan2(ay, az);
//    pitch_acc = atan2(-ax, sqrt(ay * ay + az * az));
//
//    float mx2 = mx * cos(pitch_acc) + mz * sin(pitch_acc);
//    float my2 = mx * sin(roll_acc) * sin(pitch_acc) + my * cos(roll_acc) - mz * sin(roll_acc) * cos(pitch_acc);
//    yaw_mag = atan2(-my2, mx2);
//}
//
//void complementary_predict(Vec *gyro, float dt) {
//    float gx = gyro->r[0];
//    float gy = gyro->r[1];
//    float gz = gyro->r[2];
//
//    float alpha = 0.9; // Higher = trust gyro more, lower = trust accel more
//
//    roll = alpha * (roll + gx * dt / 57.2) + (1.0 - alpha) * roll_acc;
//    pitch = alpha * (pitch + gy * dt / 57.2) + (1.0 - alpha) * pitch_acc;
//    yaw = alpha * (yaw + gz * dt / 57.2) + (1.0 - alpha) * yaw_mag;
//}
//
//QUAT_ALLOC_STATIC(attitude);
//void complementary_thread() {
//    uint64_t next_mag_time = now();
//    uint32_t last_time = now_high_accuracy();
//    while (1) {
//        volatile uint64_t next_time = now() + 5 * MILLISECONDS;
//        if (next_mag_time < now()) { // we only get this every 50 milliseconds, so setting the value
//                                     // more ofter just wastes computation
//            complementary_update(&LSM9DS1_acc, &LSM9DS1_mag);
//
//            next_mag_time = now() + 50 * MILLISECONDS;
//        }
//        complementary_predict(&LSM9DS1_gyro_filtered, 0.005);
//        //os_printf("%f %f %f \n", roll * 57.2, pitch * 57.2, yaw * 57.2);
//
//        quat_from_rpy(&attitude, roll, pitch, yaw);
//        os_printf("Attitude: ");
//        quat_print(&attitude);
//
//        sleep_until(next_time);
//    }
//}
