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
#include "math/vector.h"



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
    //scheduler_enable();
    
    Vector v1(3);
    v1[0] = 1;
    v1[1] = -1;
    v1[2] = 0;
    v1.print();
    Vector v2 = v1;
    //os_printf("Dot product: %f \n", v1 * v2);
    v2 = v2.normalize();
    v2.print();
    
    uint64_t cnt = 0;

    while (1) {
    }
}
