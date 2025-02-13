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
#include <stm32f4xx.h>


void idle1_thread () {
  uint64_t cnt = 0;
  while (1) {
    cnt++; 
    os_printf("id1 %f \n", 1.12);
    sleep(1 * SECONDS);
  }
}

void idle2_thread () {
  uint64_t cnt = 0;
  while (1) {
    cnt++; 
    os_printf("id2 \n");
    sleep(1 * SECONDS);
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
    register_thread_auto(&idle_thread, 128, 0, "idle_thread");

    // User Threads are defined here
    //uint8_t res = LSM9DS1_read_acc_and_gyro_register(LSM9DS1_STATUS_REG)[0];
    //os_printf("res: %d \n",res); 
    register_thread_auto(&idle1_thread);
    register_thread_auto(&idle2_thread);
    // End of user thread definitions

    print_thread_info();
    // start system
    scheduler_enable();
    
    uint64_t cnt = 0;

    while (1) {
    }
}
