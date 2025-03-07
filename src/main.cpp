#include <cstdint>
#include <stdint.h>
#include "communication/i2c.h"
#include "globals.h"
#include "hw_init.h"
#include "krnl/scheduler.h"
#include "krnl/thread.h"
#include "krnl/mem.h"
#include "communication/usart.h"
#include "communication/LSM9DS1.h"
#include "ekf/ekf.h"
#include "hal/hw_specific.h"

volatile void t_thred() {
    static I2C_state_information info_send;
    uint32_t data[10] = {};
    info_send.state = Recieving;
    info_send.device_adress_write = LSM9DS1_ACC_AND_GYRO_WRITE;
    info_send.device_adress_recieve = LSM9DS1_ACC_AND_GYRO_READ;
    info_send.device_subadress = LSM9DS1_WHO_AM_I;
    info_send.recieve_bytes = 1;
    info_send.data = data;
    while (1) {
        i2c_handle(&info_send);
    }
}

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
    //register_thread_auto(&LSM9DS1_thread, 2000, STD_THREAD_PRIORITY, "LSM9DS1_thread");
    //register_thread_auto(&attitude_thread, 3000, STD_THREAD_PRIORITY + 1, "attitude_thread");
    register_thread_auto(&t_thred, 3000, STD_THREAD_PRIORITY + 1, "t_thread");
    // End of user thread definitions

    print_thread_info();
    // start system
    scheduler_enable();

    idle_thread();
    while(1) {
        //OS_WARN("Scheduler didn't start!");
    }
}
