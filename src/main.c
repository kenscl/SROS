#include <stdint.h>

#include "communication/usart.h"
#include "hal/hw_specific.h"
#include "hw_init.h"
#include "krnl/mem.h"
#include "krnl/scheduler.h"
#include "krnl/thread.h"
#include "communication/SPI.h"
#include "sensors/LSM9DS1.h"
#include "attitude/ekf.h"
#include "attitude/complementary.h"

int main(void) {
    // system config
    clock_init();
    mem_init();
    enable_usart();
    scheduler_init();
    interrupt_init();
    miscellaneous_init();

    // default run parameters
    print_welcome_msg();
    register_thread_auto(&idle_thread, 10, 0, "idle_thread");

    // User Threads are defined here
    register_thread_auto(&SPI_thread, 500, 10, "SPI_thread");
    register_thread_auto(&LSM9DS1_thread, 1000, 10, "LSM9DS1_thread");
    //register_thread_auto(&attitude_thread, 4000, 10, "EKF");
    register_thread_auto(&complementary_thread, 1000, 10, "c_thread");
    // End of user thread definitions


    print_thread_info();

    // start system
    scheduler_enable();

    while (1) {
        //OS_WARN("Scheduler didn't start!");
    }
}
