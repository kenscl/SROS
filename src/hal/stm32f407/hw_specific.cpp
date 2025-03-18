#include "../hw_specific.h"
#include "../../globals.h"
#include "../../communication/usart.h"
#include <stm32f4xx.h>

void os_interrupt_enable() {
    __asm volatile ("CPSIE         I \n");
}

void os_interrupt_disable() {
    __asm volatile ("CPSID         I \n");
}

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

  void dma1_stream6_handler(void) {
      volatile uint32_t status = DMA1->HISR;
      if (DMA1->HISR & DMA_HISR_TCIF6) { // Check transfer complete flag
        DMA1->HIFCR |= DMA_HIFCR_CTCIF6; // Clear flag
      }
      if (DMA1->HISR & DMA_HISR_TEIF6) { // Check transfer complete flag
        DMA1->HIFCR |= DMA_HIFCR_CTEIF6; // Clear flag
      }
  }
}

volatile void idle_thread () {
  uint64_t cnt = 0;
  while (1) {
    cnt++; 
    if (cnt % 1000000 == 0)
    GPIOD->ODR ^= (1 << 13); // Toggle Orange LED
  }
}

volatile void one_second_thread () {
  uint64_t cnt = 0;
  while (1) {
      GPIOD->ODR ^= (1 << 15); // Toggle Blue LED
      sleep (1 * SECONDS);
  }
}

void dma_init() {
    RCC->AHB1ENR |= (1 << 21); // enable DMA1 clock
    DMA1_Stream6->CR &= ~(1 << 0); // disable
    while (DMA1_Stream6->CR & (1 << 0));
    DMA1_Stream6->PAR  = (uint32_t) &USART2->DR; // set adress of send register
    DMA1_Stream6->CR |= (1 << 10); // memory increment mode
    DMA1_Stream6->CR |= (1 << 4); // transfer interrupt enable
    DMA1_Stream6->CR |= (1 << 2); // transfer interrupt error enable
    
    DMA1_Stream6->CR |= (0b10 << 16); // high priority
    DMA1_Stream6->CR &= ~(0b11 << 13); // 8 bit
    DMA1_Stream6->CR &= ~(0b11 << 6);
    DMA1_Stream6->CR |= (0b01 << 6); // direction

    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

void TIM2_init() {
    RCC->APB1ENR |=  (1 << 0); // turn on clock form timer
    TIM2->PSC = 83;
    TIM2->ARR = 0xFFFFFFFF; // max
    TIM2->CR1 |= (1 << 0); // enable
}

uint32_t now_high_accuracy() {
    return TIM2->CNT;
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
   
    RCC->AHB1ENR |= (1 << 21); // enable DMA1 clock
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    register_thread_auto(&one_second_thread, 128, STD_THREAD_PRIORITY, "1_second_thread");
    TIM2_init();
}


void print_welcome_msg() {
    os_putstr("\n\n|-----------------------------------------| \n");
    os_putstr("System initialisation done. \n");
    os_putstr("Simple Realtime Operating System 0.2 \n");
    os_putstr("Compiled for STM32f407G \n");
    os_putstr("\n");
}
