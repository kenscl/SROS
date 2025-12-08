#include <stdint.h>

#include "communication/usart.h"
#include "globals.h"
#include "hal/hw_specific.h"
#include "hw_init.h"
#include "krnl/mem.h"
#include "krnl/scheduler.h"
#include "krnl/thread.h"
#include "communication/SPI.h"
#include "math/matrix.h"
#include "sensors/LSM9DS1.h"
#include "attitude/ekf.h"
#include "attitude/complementary.h"
#include "actuators/motors.h"

int main(void) {
    // system config
    scheduler_init();
    enable_usart();
    hal_init();
    interrupt_init();
    miscellaneous_init();

    // default run parameters
    print_welcome_msg();
    OS_THREAD(idle_thread, 200, 0, "idle_thread");

    // User Threads are defined here

    OS_THREAD(SPI_thread, 1000, 10, "SPI_thread");
    OS_THREAD(LSM9DS1_thread, 2000, 10, "LSM9DS1_thread");
    //OS_THREAD(attitude_thread, 2000, 10, "EKF_thread");
    OS_THREAD(complementary_thread, 500, 10, "complementary_thread");

    // End of user thread definitions

    print_thread_info();


    // start system
    scheduler_enable();


    while (1) {
        //OS_WARN("Scheduler didn't start!");
    }
}
