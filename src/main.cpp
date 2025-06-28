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
#include "attitude/ekf.h"
#include "attitude/complementary_filter.h"


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
    register_thread_auto(&attitude_thread, 3000, STD_THREAD_PRIORITY + 1, "attitude_thread");
    //register_thread_auto(&attitude_thread_complementary_filter, 3000, STD_THREAD_PRIORITY + 1, "attitude_thread");
    // End of user thread definitions
    SPI_init();

    print_thread_info();
    // start system
    scheduler_enable();


    //idle_thread();
    while(1) {
        //OS_WARN("Scheduler didn't start!");
    }
}
