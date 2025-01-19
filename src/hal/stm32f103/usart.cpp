#include <stdint.h>
#include <stm32f1xx.h>
#include "stm32f103xb.h"
#include <stdio.h>
#include <stdarg.h>
#include "../../krnl/mem.h"

void enable_usart1(){

    volatile uint32_t dummy;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    dummy = RCC->APB1ENR;
    dummy = RCC->APB1ENR;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    dummy = RCC->APB1ENR;
    dummy = RCC->APB1ENR;

    GPIOA->CRH |= GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1;
    GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10_0 | GPIO_CRH_CNF10_1);

    USART1->CR1 &= ~USART_CR1_M;   // 8 data bits
    USART1->CR2 &= ~USART_CR2_STOP; // 1 stop bit
    USART1->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS); // No parity
    USART1->BRR = 0x271; //115200 Baud @ 72MHz yes its odd i know
    USART1->CR1 |= USART_CR1_UE | USART_CR1_TE;

}

void usart_write(USART_TypeDef *usart, char c) {
    if (c == '\n') {
        usart->DR = '\r';
        while (!(usart->SR & USART_SR_TC));
    }
    usart->DR = c;
    while (!(usart->SR & USART_SR_TC));
}


int get_int_size (int i) {
    int cnt = 0;
    while (1) {
        i = i / 10; 
        cnt ++;
        if (i == 0) {
            return cnt;
        }
    }
    return 0;
}

void os_putstr(const char *s) {
	while (*s != '\000') {
		usart_write(USART1, *s);
		if (*s == '\n') {
		}
		s++;
	}
}

void os_putchar(char c) {
    usart_write(USART1, c);
}

void os_putint(int num) {
    if (num == 0) {
        os_putchar('0');
        return;
    }
    int is_negative = 0;
    int i = 0;
    int size = get_int_size(num);
    if (num < 0) {
        is_negative = 1;
        num = - num;
    }
    
    char* result = (char *) os_alloc(size + is_negative + 1);
    int it = 0;
    while (num > 0 && it < 1000) {
        it++;
        result[i++] = (num % 10) + '0';
        num /= 10;
    }

    if (is_negative == 1) {
        result[i++] = '-';
        size++;
    }

    for (int j = 0; j < i / 2; j++) {
        char temp = result[j];
        result[j] = result[i - j - 1];
        result[i - j - 1] = temp;
    }
    for (int i = 0; i < size; ++i) {
        os_putchar(result[i]);
    }

    os_free(result);
}

void os_putf(float f) { // broken
}


void os_printf(const char* format, ... ) {
    __disable_irq();
    va_list args;
    va_start(args, format);

    char c;
    const char *str;
    int num;

    for (int i = 0; format[i] != '\0'; i++) {
        if (format[i] == '%') {
            i++; 

            switch (format[i]) {
            case 'd': 
                num = va_arg(args, int);
                os_putint(num);
                break;

            case 'f': 
                num = va_arg(args, int);
                os_putf(num);
                break;

            case 's':
                str = va_arg(args, const char*);
                os_putstr(str);
                break;

            case 'c':
                c = (char) va_arg(args, int);
                os_putchar(c);
                break;

            default:
                os_putchar('%');
                os_putchar(format[i]);
                break;
            }
        } else {
            os_putchar(format[i]);
        }
    }
    os_putchar('\0');
    va_end(args);
    __enable_irq();
}
