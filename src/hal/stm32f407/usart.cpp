#include <stdint.h>
#include <stm32f4xx.h>
#include <stdio.h>
#include <stdarg.h>
#include "../../krnl/mem.h"

// ill use usart 2 / pa2 tx, pa3 rx
void enable_usart(){
    //// enable usart2 clock
    //RCC->APB1ENR |= (1 << 17);
    //// enable gpioa clock
    //RCC->AHB1ENR |= (1 << 0);
    //// enable pa2, pa3 alternate function mode
    //GPIOA->MODER |= (0b10 << 8);
    //GPIOA->MODER |= (0b10 << 6);
    ////Select the type, pull-up/pull-down and output speed via the GPIOx_OTYPER, GPIOx_PUPDR and GPIOx_OSPEEDR registers, respectively
    //GPIOA->OTYPER &= ~(1 << 2);
    //GPIOA->OTYPER |= (1 << 3);
    //GPIOA->PUPDR |= (0b00 << 4);
    //GPIOA->PUPDR |= (0b01 << 6);
    //// Connect the I/O to the desired AFx
    //GPIOA->AFRL |= (0b0111 < 8);
    //GPIOA->AFRL |= (0b0111 < 12);


    
}

void os_putf(float f) { // broken
}


void os_printf(const char* format, ... ) {
    //__disable_irq();
    //va_list args;
    //va_start(args, format);

    //char c;
    //const char *str;
    //int num;

    //for (int i = 0; format[i] != '\0'; i++) {
    //    if (format[i] == '%') {
    //        i++; 

    //        switch (format[i]) {
    //        case 'd': 
    //            num = va_arg(args, int);
    //            os_putint(num);
    //            break;

    //        case 'f': 
    //            num = va_arg(args, int);
    //            os_putf(num);
    //            break;

    //        case 's':
    //            str = va_arg(args, const char*);
    //            os_putstr(str);
    //            break;

    //        case 'c':
    //            c = (char) va_arg(args, int);
    //            os_putchar(c);
    //            break;

    //        default:
    //            os_putchar('%');
    //            os_putchar(format[i]);
    //            break;
    //        }
    //    } else {
    //        os_putchar(format[i]);
    //    }
    //}
    //os_putchar('\0');
    //va_end(args);
    //__enable_irq();
}
