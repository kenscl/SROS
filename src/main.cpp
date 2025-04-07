#include <cstdint>
#include <stdint.h>
#include "communication/LSM9DS1.h"
#include "globals.h"
#include "hw_init.h"
#include "krnl/scheduler.h"
#include "krnl/thread.h"
#include "krnl/mem.h"
#include "communication/usart.h"
#include "hal/hw_specific.h"
#include "communication/LSM9DS1.h"
#include "communication/SPI.h"
#include "ekf/ekf.h"
#include "stm32f407xx.h"

Vec3 gyro_m[2];
Vec3 acc_m[2];
Vec3 mag_m[2];

int main (void) {
    // system config
    clock_init();
    mem_init();
    enable_usart();
    scheduler_init();
    interrupt_init();
    miscellaneous_init();

    // default run parameters
    print_welcome_msg();
    register_thread_auto(&idle_thread, 200, 0, "idle_thread");

    // User Threads are defined here
    register_thread_auto(&LSM9DS1_thread, 2000, STD_THREAD_PRIORITY, "LSM9DS1_thread");
    register_thread_auto(&SPI_thread, 500, 10, "SPI_thread");
    register_thread_auto(&attitude_thread, 3000, STD_THREAD_PRIORITY, "attitude_thread");
    // End of user thread definitions
    SPI_init();

    print_thread_info();
    // start system
    scheduler_enable();

    //gyro_m[0][0] = 0.0354301847466;
    //gyro_m[0][1] = 0.0427605688065; 
    //gyro_m[0][2] = 0.0839939681716;

    //gyro_m[1][0] = 0.0433714307313;
    //gyro_m[1][1] = 0.041233402291;
    //gyro_m[1][2] = 0.0760527221869;

    //acc_m[0][0] = 0.131455004215;
    //acc_m[0][1] = 0.0590480007231;
    //acc_m[0][2] = -0.984296023846;

    //acc_m[1][0] = 0.130173996091;
    //acc_m[1][1] = 0.0571570023894;
    //acc_m[1][2] = -0.984296023846;

    //mag_m[0][0] = 0.171679988503;
    //mag_m[0][1] = 0.0350400023162;
    //mag_m[0][2] = 0.454560011625;

    //mag_m[1][0] = 0.175359994173;
    //mag_m[1][1] = 0.0375999994576;
    //mag_m[1][2] = 0.454400002956;

    //EKF ekf;
    //ekf.init(gyro_m, acc_m, mag_m);
    //ekf.predict(gyro_m[0], 0.004940032);
    //ekf.update_acc(acc_m[0]);
    //ekf.update_mag(mag_m[0]);
    //ekf.update();
    //ekf.predict(gyro_m[1], 0.005022464);
    //ekf.update_acc(acc_m[1]);
    //ekf.update_mag(mag_m[1]);
    //ekf.update();

    //idle_thread();
    while(1) {
        //OS_WARN("Scheduler didn't start!");
    }
}
