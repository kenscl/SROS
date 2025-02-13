#include <cstdint>
#include <stdint.h>
#include <stm32f4xx.h>
#include <stdio.h>
#include <stdarg.h>
#include "../../krnl/mem.h"

// ill use usart 2 / pa2 tx, pa3 rx
void enable_usart(){
    // enable usart2 clock
    RCC->APB1ENR |= (1 << 17);
    // enable gpioa clock
    RCC->AHB1ENR |= (1 << 0);
    // enable pa2, pa3 alternate function mode
    GPIOA->MODER |= (0b10 << 6);
    GPIOA->MODER |= (0b10 << 4);
    //Select the type, pull-up/pull-down and output speed via the GPIOx_OTYPER, GPIOx_PUPDR and GPIOx_OSPEEDR registers, respectively
    GPIOA->OTYPER &= ~(1 << 2);
    GPIOA->OTYPER &= ~(1 << 3);
    GPIOA->PUPDR &= ~(0b11 << 4);
    GPIOA->PUPDR |= (0b01 << 6);
    GPIOA->OSPEEDR |= (3 << 4);
    // Connect the I/O to the desired AFx
    GPIOA->AFR[0] &= ~((0b1111 << 8) | (0b1111 << 12));
    GPIOA->AFR[0] |= (0b0111 << 8);
    GPIOA->AFR[0] |= (0b0111 << 12);
    // 8 data bits
    USART2->CR1 &= ~(1 << 12);
    // 1 stop bits
    USART2->CR2 &= ~(0b11 << 12);
    // No parity
    USART2->CR1 &= ~(1 << 10);
    //115200 Baud
    // sysclk is 168, apb1 prescaler is 4, so usart clock should be 42mhz, so BRR = usart_clk / 115200 = 364,6
    USART2->BRR = 364; 
    USART2->CR1 |= (1 << 13);
    USART2->CR1 |= (1 << 2) | (1 << 3);
    

}


void os_putchar(char c) {
    USART2->DR = c;
    while (!(USART2->SR & USART_SR_TC));
}

void os_putstr(const char *s) {
	while (*s != '\000') {
		os_putchar(*s);
		s++;
	}
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

void os_printf(const char* format, ... ) {
    GPIOD->ODR |= (1 << 12);
    __disable_irq();
    va_list args;
    va_start(args, format);

    char c;
    const char *str;
    double num_f;
    int i_part;
    int f_part;
    for (int i = 0; format[i] != '\0'; i++) {
      if (format[i] == '%') {
        i++;

        switch (format[i]) {
        case 'd':
          int num_i;
          num_i = va_arg(args, int);
          os_putint(num_i);
          break;

        case 'f':
          num_f = va_arg(args, double);
          i_part = num_f;
          f_part = (num_f - i_part) * 10e6;
          os_putint(i_part);
          os_putchar('.');
          os_putint(f_part);
          break;

        case 's':
          str = va_arg(args, const char *);
          os_putstr(str);
          break;

        case 'c':
          c = (char)va_arg(args, int);
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
    GPIOD->ODR &= ~(1 << 12);
    __enable_irq();
}
