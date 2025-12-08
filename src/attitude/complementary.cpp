#include "complementary.h"
#include "../config.h"
float roll_acc = 0;
float pitch_acc = 0;
float yaw_mag = 0;
float pitch =0;
float roll =0;
float yaw =0;

float pitch_gyro =0;
float roll_gyro =0;
float yaw_gyro =0;

void complementary_update(Vec3 *acc, Vec3 *mag) {
    float ax = (*acc)[0];
    float ay = (*acc)[1];
    float az = (*acc)[2];
    float mx = (*mag)[0];
    float my = (*mag)[1];
    float mz = (*mag)[2];
    roll_acc = atan2(ay, az);
    pitch_acc = atan2(-ax, sqrtf(ay * ay + az * az));

    float mx2 = mx * cosf(pitch_acc) + mz * sinf(pitch_acc);
    float my2 = mx * sinf(roll_acc) * sinf(pitch_acc) + my * cosf(roll_acc) -
                mz * sinf(roll_acc) * cosf(pitch_acc);
    yaw_mag = atan2f(-my2, mx2);
}

void complementary_predict(Vec3 *gyro, float dt) {
    float gx = (*gyro)[0];
    float gy = (*gyro)[1];
    float gz = (*gyro)[2];

    float alpha = 0.9; // Higher = trust gyro more, lower = trust accel more

    roll = alpha * (roll + gx * dt / 57.2) + (1.0 - alpha) * roll_acc;
    pitch = alpha * (pitch + gy * dt / 57.2) + (1.0 - alpha) * pitch_acc;
    yaw = alpha * (yaw + gz * dt / 57.2) + (1.0 - alpha) * yaw_mag;
}

volatile void complementary_thread() {
    uint64_t next_mag_time = now();
    uint32_t last_time = now_high_accuracy();
    static Quat attitude;
    while (1) {
	volatile uint64_t next_time = now() + 5 * MILLISECONDS;
	if (next_mag_time < now()) {

	    complementary_update(&LSM9DS1_acc, &LSM9DS1_mag);

            next_mag_time = now() + 20 * MILLISECONDS;
        }
        complementary_predict(&LSM9DS1_gyro_filtered, 0.005);


	attitude = Quat(roll, pitch, yaw);

#if PRINT_ATTITUDE == 1
	os_printf("Attitude: ");
        attitude.print_bare();
#endif

        sleep_until(next_time);
    }
}
