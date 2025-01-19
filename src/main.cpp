#include <stm32f1xx.h>
#include <system_stm32f1xx.h>
#include <stdint.h>
#include <stdio.h>
#include "hw_init.h"
#include "krnl/scheduler.h"
#include "krnl/thread.h"
#include "krnl/mem.h"
#include "communication/usart.h"
#include "communication/usart.h"
#include "communication/i2c.h"
#include "communication/LSM9DS1.h"
#include "hal/hw_specific.h"
#define STM32F100xB
#define SYSCLK_FREQ_72MHz  72000

void idle_thread () {
  uint64_t cnt = 0;
  while (1) {
    cnt++; 
  }
}

void idle1_thread () {
  uint64_t cnt = 0;
  while (1) {
    cnt++; 
    os_printf("id1 \n");
  }
}

void idle2_thread () {
  uint64_t cnt = 0;
  while (1) {
    cnt++; 
    os_printf("id2 \n");
  }
}
void print_welcome_msg() {
    os_printf("\n\n|-----------------------------------------| \n");
    os_printf("System initialisation done. \n");
    os_printf("Simple Realtime Operating System 0.2 \n");
    os_printf("\n");
}

int main (void) {
    // system config
    mem_init();
    enable_usart1();
    scheduler_init();
    clock_init();
    interrupt_init();

    // default run parameters
    print_welcome_msg();
    register_thread_auto(&idle_thread, 128, 0, "idle_thread");

    // User Threads are defined here
    uint8_t res = LSM9DS1_read_acc_and_gyro_register(LSM9DS1_STATUS_REG)[0];
    os_printf("res: %d \n",res); 
    register_thread_auto(&idle1_thread);
    register_thread_auto(&idle2_thread);
    // End of user thread definitions

    print_thread_info();
    // start system
    scheduler_enable();
    while (1);
}

