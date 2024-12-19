#include <stm32f1xx.h>
void clock_init(){
    FLASH->ACR |= FLASH_ACR_LATENCY_2;
    RCC->CFGR |= RCC_CFGR_PPRE1_2;
    RCC->CFGR |= RCC_CFGR_PLLXTPRE_HSE;
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    RCC->CFGR |= RCC_CFGR_PLLSRC;
    RCC->CFGR |= RCC_CFGR_PLLMULL9;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));

}
