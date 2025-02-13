#include "../hw_specific.h"
#include "../../krnl/scheduler.h"
#include "../../communication/usart.h"
#include <stm32f1xx.h>

extern "C" {
  __attribute__((naked)) void pendsv_handler(void) {

    __asm volatile (
                    "CPSID         I \n"

                    "LDR           r1,=current_thread \n"
                    "LDR           r1,[r1,#0x00] \n"
                    "CBZ           r1,_restore \n"

                    "stmdb	sp!, {r4, r5, r6, r7, r8, r9, r10, r11, r14} \n"

                    "LDR           r1,=current_thread \n"
                    "LDR           r1,[r1,#0x00] \n"
                    "STR           sp,[r1,#0x00] \n"

                    "_restore: \n"
                    "bl schedule \n"
                    "LDR           r1,=current_thread \n"
                    "LDR           r1,[r1,#0x00] \n"
                    "LDR           sp,[r1,#0x00] \n"

                    "ldmia	sp!, {r4, r5, r6, r7, r8, r9, r10, r11, r14} \n"

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
  }
}

void miscellaneous_init() {
}

void print_welcome_msg() {
    os_printf("\n\n|-----------------------------------------| \n");
    os_printf("System initialisation done. \n");
    os_printf("Simple Realtime Operating System 0.2 \n");
    os_printf("Compiled for STM32f103\n");
    os_printf("\n");
}
