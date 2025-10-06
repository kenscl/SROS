#include "../hw_specific.h"
#include "../../krnl/scheduler.h"
#include <stm32f1xx.h>
#include "../../communication/usart.h"

__attribute__((naked)) void pendsv_handler(void) {

  __asm volatile("CPSID         I \n"

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

void systick_handler() {
  ticks++;
  if (!sched_on)
    return;
  SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

volatile uint32_t tim2_overflow = 0;

void tim2_handler(void) {
  if (TIM2->SR & TIM_SR_UIF) {
    TIM2->SR &= ~TIM_SR_UIF;
    tim2_overflow++;
  }
}

volatile void idle_thread () {
  uint64_t cnt = 0;
  while (1) {
    cnt++;
  }
}

void TIM2_init() {
    RCC->APB1ENR |=  (1 << 0); // turn on clock form timer
    TIM2->PSC = 83;
    TIM2->ARR = 0xFFFFFFFF; // max
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= (1 << 0); // enable
    NVIC_EnableIRQ(TIM2_IRQn);
}

uint64_t now_high_accuracy() {
    uint64_t high, low;

    // Double-read for atomicity
    do {
        high = tim2_overflow;
        low = TIM2->CNT;
    } while (high != tim2_overflow);

    return high * 4294967295 + low;
}

void miscellaneous_init() { TIM2_init(); }

void print_welcome_msg() {
    os_printf("\n\n|-----------------------------------------| \n");
    os_printf("System initialisation done. \n");
    os_printf("Simple Realtime Operating System 0.2 \n");
    os_printf("Compiled for STM32f103\n");
    os_printf("\n");
}
