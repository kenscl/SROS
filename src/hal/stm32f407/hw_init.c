#include <stm32f4xx.h>

void clock_init(){
    // 1. Increase CPU frequency to max (page 82)
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

    // performance stuff
    FLASH->ACR |= (1 << 9);            // Instruction Cache Enable
    FLASH->ACR |= (1 << 10);            // Data Cache Enable

    // Enable Prefetch buffer
    FLASH->ACR |= (1 << 8);

    int dummy = FLASH->ACR; // ensure that latency is set


    RCC->CR |= (1 << 16);                  // Turn on HSE
    while (!(RCC->CR & (1 << 17)));       // Wait for HSE to stabilize
    // 168 MHz = 8 MHz (HSe) * 336 (PLLN) / (8 (PLLP) * 2 (PLLM))
    RCC->PLLCFGR = 0;                         // Reset PLLCFGR
    RCC->PLLCFGR |= (1 << 22); // select HSE as pllsrc
    RCC->PLLCFGR |= (336 << 6); // select vco ouput scaler
    RCC->PLLCFGR |= (8 << 0); // pllm 
    RCC->PLLCFGR &= ~(0b11 << 16); // pllp 
    RCC->PLLCFGR |= (0b00 << 16); // pllp 

    RCC->CR |= (1 << 24);                  // Turn on HSE
    while (!(RCC->CR & (1 << 25)));       // Wait for HSE to stabilize
    RCC->CFGR |= (0b0000 << 4); // set AHB prescaler to 1
    RCC->CFGR |= (0b101 << 10); // set APB1 prescaler to 4
    RCC->CFGR |= (0b000 << 13); // set APB2 prescaler to 1

    // use pll as sysclock source 
    RCC->CFGR |= (0b10 << 0);
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait for PLL to be used
}


void interrupt_init() {
    // configure systick handler
    // rvr = (cpu clock / 1000) - 1 = 168 MHz / 1000 - 1 = 167999
    SysTick->LOAD = 167999;
    SysTick->VAL = 0;
    SysTick->CTRL |= (1 << 2); // cpu clock source
    SysTick->CTRL |= (1 << 1); // set status to pending on count == 0 
    
    // set pendsv prio to 0xff and systick to 0x00
    SCB->SHP[10] = 0xff;
    SCB->SHP[11] = 0x05;
    //NVIC_SetPriority(DMA1_Stream6_IRQn, 3);
    // enable systick and pendsv
    SysTick->CTRL |= (1 << 0); // enable  
    // enable global interrupts

    // some more work is needed to preserve fpu context on expetion
    FPU->FPCCR |= (1 << 31); // Set ASPEN (Automatic State Preservation Enable)
    FPU->FPCCR |= (1 << 30); // Set LSPEN (Lazy State Preservation Enable)
    __enable_irq();
}
