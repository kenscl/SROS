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

//MAT_ALLOC_STATIC(TST, 10, 10);
//MAT_ALLOC_STATIC(TST2, 10, 10);
//MAT_ALLOC_STATIC(RES, 10, 10);
//void test() {
//    mat_identity(&TST);
//    mat_identity(&TST2);
//    uint64_t next_time = 0;
//    while (1) {
//        //int res = mat_inverse(&TST, &TST2);
//        mat_mult(&TST, &TST2, &RES);
//        for (int i = 0; i < 9; ++i) {
//            if (fabs(RES.r[i + 10 * i] - 1.0) > 1e-4) {
//            os_printf("ERROR %f\n", RES.r[i + 10 * i]);
//            mat_print(&TST2);
//            mat_print(&RES);
//            }
//        }
//        if (now() > next_time) {
//            next_time = now() + 1 * SECONDS;
//        }
//    }
//
//}


int main(void) {
    // system config
    mem_init();
    scheduler_init();
    hal_init();
    interrupt_init();
    miscellaneous_init();

    // default run parameters
    print_welcome_msg();
    register_thread_auto(&idle_thread, 10, 0, "idle_thread");

    // User Threads are defined here
    //register_thread_auto(&SPI_thread, 1500, 10, "SPI_thread");
    //register_thread_auto(&LSM9DS1_thread, 2000, 10, "LSM9DS1_thread");
    //register_thread_auto(&attitude_thread, 4000, 10, "EKF");
    //register_thread_auto(&test, 4000, 10, "EKF2");
    //register_thread_auto(&complementary_thread, 1000, 10, "c_thread");
    // End of user thread definitions


    print_thread_info();

    // start system
    scheduler_enable();

    while (1) {
        //OS_WARN("Scheduler didn't start!");
    }
}
