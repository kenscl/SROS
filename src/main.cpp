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

uint32_t idle_stack[200];
os_pcb idle_pcb;

uint32_t spi_stack[1500];
os_pcb spi_pcb;

uint32_t lsm_stack[2000];
os_pcb lsm_pcb;

uint32_t ekf_stack[4000];
os_pcb ekf_pcb;

uint32_t motor_stack[2000];
os_pcb motor_pcb;
int main(void) {
    // system config
    //mem_init();
    scheduler_init();
    enable_usart();
    hal_init();
    interrupt_init();
    miscellaneous_init();

    // default run parameters
    print_welcome_msg();
    register_thread_auto(&idle_thread, 200, idle_stack, &idle_pcb, 0, "idle_thread");

    // User Threads are defined here
    register_thread_auto(&SPI_thread, 1500, spi_stack, &spi_pcb, 10, "SPI_thread");
    register_thread_auto(&LSM9DS1_thread, 2000, lsm_stack, &lsm_pcb, 10, "LSM9DS1_thread");
    register_thread_auto(&attitude_thread, 4000, ekf_stack, &ekf_pcb, 10, "EKF");
    //register_thread_auto(&test, 4000, 10, "EKF2");
    register_thread_auto(&motor_thread, 2000, motor_stack, &motor_pcb, 10, "Motors");
    //register_thread_auto(&complementary_thread, 1000, 10, "c_thread");
    // End of user thread definitions

    print_thread_info();


    // start system
    scheduler_enable();


    while (1) {
        //OS_WARN("Scheduler didn't start!");
    }
}
