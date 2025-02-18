#include <cstdint>
#include <stdint.h>
#include <stdio.h>
#include "globals.h"
#include "hw_init.h"
#include "krnl/scheduler.h"
#include "krnl/thread.h"
#include "krnl/mem.h"
#include "communication/usart.h"
#include "communication/usart.h"
#include "communication/i2c.h"
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

    // User Threads are defined here
    register_thread_auto(&LSM9DS1_thread, 1000, 20, "LSM9DS1_thread");
    // End of user thread definitions

    print_thread_info();
    // start system
    scheduler_enable();
    
    uint64_t cnt = 0;

    while (1) {
    }
}
