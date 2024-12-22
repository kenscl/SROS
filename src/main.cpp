#include <stm32f1xx.h>
#include <system_stm32f1xx.h>
#include <stdint.h>
#include <stdio.h>
#include "usart.h"
#include "hw_init.h"
#include "scheduler.h"
#include "thread.h"
#include "usart.h"
#include "mem.h"
#define STM32F100xB
#define SYSCLK_FREQ_72MHz  72000 

void idle_thread () {
  uint64_t cnt = 0;
  while (1) {
    cnt++; 
  }
}

void print_welcome_msg() {
    os_printf("\n\n|-----------------------------------------| \n");
    os_printf("System initialisation done. \n");
    os_printf("Simple Realtime Operating System 0.1 \n");
    os_printf("\n");
}

void interrupt_init() {
    SysTick_Config(SYSCLK_FREQ_72MHz);
    NVIC_SetPriority(PendSV_IRQn, 0xFFU);
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
    NVIC_EnableIRQ(SysTick_IRQn);
    NVIC_EnableIRQ(PendSV_IRQn); 
    __enable_irq();
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

    // End of user thread definitions

    print_thread_info();
    // start system
    scheduler_enable();
    while (1);
}
