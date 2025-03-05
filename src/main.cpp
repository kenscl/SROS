#include <cstdint>
#include <stdint.h>
#include "globals.h"
#include "hw_init.h"
#include "krnl/scheduler.h"
#include "krnl/thread.h"
#include "krnl/mem.h"
#include "communication/usart.h"
#include "communication/LSM9DS1.h"
#include "hal/hw_specific.h"


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
    register_thread_auto(&msg_thread, 200, STD_THREAD_PRIORITY, "printf_thread");

    // User Threads are defined here
    //register_thread_auto(&test_thread, 200, 10, "test_thread");
    register_thread_auto(&LSM9DS1_thread, 3000, STD_THREAD_PRIORITY, "LSM9DS1_thread");
    // End of user thread definitions

    print_thread_info();
    // start system
    scheduler_enable();

    idle_thread();
    LSM9DS1_thread();
    msg_thread();
    while(1) {
        //OS_WARN("Scheduler didn't start!");
    }
}
