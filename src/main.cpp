#include <cstdint>
#include <stdint.h>
#include "globals.h"
#include "hw_init.h"
#include "krnl/scheduler.h"
#include "krnl/thread.h"
#include "krnl/mem.h"
#include "communication/usart.h"
#include "hal/hw_specific.h"
#include "communication/SPI.h"


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
    register_thread_auto(&SPI_thread, 500, 10, "SPI_thread");
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
