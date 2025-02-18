#include "../hw_specific.h"
#include "../../globals.h"
#include "../../communication/usart.h"
#include <stm32f4xx.h>

extern "C" {
  __attribute__((naked)) void pendsv_handler(void) {

    __asm volatile (
                    "CPSID         I \n"

                    "LDR           r1,=current_thread \n"
                    "LDR           r1,[r1,#0x00] \n"
                    "CBZ           r1,_restore \n"

                    "mrs r0, psp \n"
                    "stmdb	r0!, {r4, r5, r6, r7, r8, r9, r10, r11, r14} \n"
                    "LDR           r1,=current_thread \n"
                    "LDR           r1,[r1,#0x00] \n"
                    "STR           r0,[r1,#0x00] \n"

                    "_restore: \n"
                    "bl schedule \n"
                    "LDR           r1,=current_thread \n"
                    "LDR           r1,[r1,#0x00] \n"
                    "LDR           r0,[r1,#0x00] \n"


                    "ldmia	r0!, {r4, r5, r6, r7, r8, r9, r10, r11, r14} \n"

                    "msr            psp,r0 \n"

                    "CPSIE         I \n"

                    "BX            lr \n");
  }

  void systick_handler()
  {
    ticks++;
    if (!sched_on) return;
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
  }
}

void idle_thread () {
  uint64_t cnt = 0;
  while (1) {
    cnt++; 
    if (cnt % 1000000 == 0)
    GPIOD->ODR ^= (1 << 13); // Toggle Orange LED
  }
}

void one_second_thread () {
  uint64_t cnt = 0;
  while (1) {
      GPIOD->ODR ^= (1 << 15); // Toggle Blue LED
      sleep (1 * SECONDS);
  }
}

void miscellaneous_init() {
    FPU->FPCCR |= (1 << 30) | (1 << 31);  // Set ASPEN and LSPEN
    SCB->CPACR |= (0xF << 20); 
    __DSB();
    __ISB();
    // led for idle [orange] toggle after 1000000 itterations
    // led for print [green] on while printing
    // led every second [blue]
    // error led [red]
    RCC->AHB1ENR |= (1 << 3);
    GPIOD->MODER &=
        ~((0b11 << (12 * 2)) | (0b11 << (13 * 2)) | (0b11 << (14 * 2)) |
          (0b11 << (15 * 2))); // Clear mode bits
    GPIOD->MODER |= ((0b01 << (12 * 2)) | (0b01 << (13 * 2)) |
                     (0b01 << (14 * 2)) | (0b01 << (15 * 2)));
    
    register_thread_auto(&one_second_thread, 128, STD_THREAD_PRIORITY, "1_second_thread");
}

void print_welcome_msg() {
    os_printf("\n\n|-----------------------------------------| \n");
    os_printf("System initialisation done. \n");
    os_printf("Simple Realtime Operating System 0.2 \n");
    os_printf("Compiled for STM32f407G \n");
    os_printf("\n");
}
