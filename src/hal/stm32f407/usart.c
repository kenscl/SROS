#include <stddef.h>
#include <stdint.h>
#include <stdint.h>
#include <stm32f4xx.h>
#include <stdio.h>
#include <stdarg.h>
#include "../../communication/usart.h"
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
    USART2->BRR |= (22 << 4);
    USART2->BRR |= 13;
    USART2->CR1 |= (1 << 13);
    USART2->CR1 |= (1 << 2) | (1 << 3);
}





#define MSG_BUFFER_SIZE 128

void os_putchar(char c) {
    USART2->DR = c;
    while (!(USART2->SR & USART_SR_TC));
}

void os_putstr(char *s) {
    size_t size = 0;
	while (s[size] != '\000') {
		size++;
	}
    msg_put(s, size);
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

char  result[16];
void os_putint(int num) {
    //char * result = (char*) os_alloc(sizeof(char) * 16);
    if (num == 0) {
        os_putstr("0\0");
        return;
    }
    char *ptr = result + 15;
    char *start = ptr;

    int is_negative = (num < 0);
    if (is_negative) {
        num = -num;
    }

    while (num > 0) {
        *--ptr = (num % 10) + '0';
        num /= 10;
    }
    if (is_negative) {
        *--ptr = '-';
    }
    uint32_t len = start - ptr;
    ptr[len] = '\0';
    os_putstr(ptr);
}

void os_putf(float num) {
          int i_part = num;
	  if (num < 0) {
	    i_part = -i_part;
	    os_putchar('-');
	    num *= -1;
	  }

          float f_part = (num - i_part);
          if (f_part < 0) {
              f_part *= -1;
          }
          os_putint(i_part);

          //f part
          os_putstr(".\0");

          int n_of_0 = 0;
          for (int i = 0; i < 7; ++i) {
              f_part = f_part * 10;
              if (f_part > 1) {}
              else n_of_0++;
          }

          if (n_of_0 < 7) {
              for (int i = 0; i < n_of_0; ++i) {
                  os_putint(0);
              }
              os_putint( (int) f_part);
          }
          else os_putint(0);
}

void os_printf(char *format, ...) {
  GPIOD->ODR |= (1 << 12);
    __disable_irq();
    va_list args;
    va_start(args, format);

    char c;
    const char *str;
    double num_f;
    int i_part;
    double f_part;

    uint16_t n_of_0 = 0;
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
          os_putf(num_f);
          break;

        case 's':
          str = va_arg(args, const char *);
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
msg_object l = {.next = (msg_object*) 0, .msg = (char*) 0, .size = 0};
msg_object * last = &l;
msg_object * head = &l;
int msg_put(char *msg, size_t size) {
    for (int i = 0;i < size; ++i) {
        os_putchar(msg[i]);
    }
    return 1;
}
