#include <stm32f1xx.h>
#define SYSCLK_FREQ_72MHz  72000
void clock_init(){
    FLASH->ACR |= FLASH_ACR_LATENCY_2;
    RCC->CFGR |= RCC_CFGR_PPRE1_2; // Apb1 = hse / 2 ?
    RCC->CFGR |= RCC_CFGR_PLLXTPRE_HSE; // hse / 2
    RCC->CR |= RCC_CR_HSEON; // turn on hse
    while (!(RCC->CR & RCC_CR_HSERDY));

    RCC->CFGR |= RCC_CFGR_PLLSRC; // set pll source to hse
    RCC->CFGR |= RCC_CFGR_PLLMULL9; // pll input * 9 = 72
    RCC->CR |= RCC_CR_PLLON; // turn pll on
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= RCC_CFGR_SW_PLL; // set clock to pll
    while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));
}

void interrupt_init() {
    SysTick_Config(SYSCLK_FREQ_72MHz);
    NVIC_SetPriority(PendSV_IRQn, 0xFFU);
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
    NVIC_EnableIRQ(SysTick_IRQn);
    NVIC_EnableIRQ(PendSV_IRQn); 
    __enable_irq();
}
