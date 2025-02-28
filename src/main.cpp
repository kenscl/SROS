#include <stdint.h>
#include "hw_init.h"
#include "krnl/scheduler.h"
#include "krnl/thread.h"
#include "krnl/mem.h"
#include "communication/usart.h"
#include "communication/usart.h"
#include "communication/LSM9DS1.h"
#include "hal/hw_specific.h"
#include "ekf/ekf.h"



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
    //register_thread_auto(&test_thread, 200, 10, "test_thread");
    register_thread_auto(&LSM9DS1_thread, 1000, 20, "LSM9DS1_thread");
    // End of user thread definitions

    print_thread_info();
    // start system
    //scheduler_enable();

    static EKF ekf;
    static Vec3 mag[100], acc[100], gyro[100];
    
    for (int i = 0; i < 50; ++i) {
        gyro[i][0] = 0.0001;
        mag[i][0] = 1;
        acc[i][2] = -1.02;
    }
    for (int i = 50; i < 100; ++i) {
        gyro[i][0] = 0.001;
        mag[i][0] = 0.5;
        mag[i][1] = 0.5;
        acc[i][2] = -0.8;
        acc[i][2] = -0.2;
    }
    ekf.init(gyro, acc, mag);
    acc[0][0] = 0.1;
    ekf.update_acc(acc[0]);
    ekf.update_mag(mag[0]);
    for (int i = 0; i < 1000; ++i) {
      ekf.predict(gyro[0], 0.0001);
      ekf.update();
      ekf.x.print();
    }
}
