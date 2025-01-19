#include "../hw_specific.h"
#include "../../krnl/scheduler.h"
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
