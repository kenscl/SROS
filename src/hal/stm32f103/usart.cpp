#include <cstddef>
#include <stdint.h>
#include <stm32f1xx.h>
#include "stm32f103xb.h"
#include <stdio.h>
#include <stdarg.h>
#include "../../communication/usart.h"
#include "../../krnl/mem.h"

void enable_usart(){
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

#define MSG_BUFFER_SIZE 128

void os_putchar(char c) {
    USART1->DR = c;
    while (!(USART1->SR & USART_SR_TC));
}

void os_putstr(char *s) {
    size_t size = 0;
	while (s[size] != '\000') {
		size++;
	}
    msg_put(s, size);
}

void os_putstr(char *s, size_t size) {
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
        os_putstr("0", 1);
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
    os_putstr(ptr, len);
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
          os_putstr(".", 1);

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
msg_object l = {.next = (msg_object*) nullptr, .msg = (char*) nullptr, .size = 0};
msg_object * last = &l;
msg_object * head = &l;
int msg_put(char *msg, size_t size) {
    for (int i = 0;i < size; ++i) {
        os_putchar(msg[i]);
    }
    return 1;
}
