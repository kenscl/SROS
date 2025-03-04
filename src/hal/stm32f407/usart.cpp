#include <cstddef>
#include <cstdint>
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
    USART2->CR3 |= (1 << 7); // enable dma
    USART2->CR3 |= (1 << 6); // enable dma
    USART2->CR3 |= USART_CR3_DMAT;
}




void USART_put_dma(char *data, size_t size) {
    while (DMA1_Stream6->CR & (1 << 0) == 0); // wait till usart is free
    DMA1_Stream6->CR = 0; // clear dma register
    while(DMA1_Stream6->CR & (1 << 0));

    DMA1_Stream6->CR |= (0b100 << 25); // channel4
    DMA1_Stream6->CR |= (1 << 4); // transfer complete interrupt
    DMA1_Stream6->CR |= DMA_SxCR_MINC;  // memory increment mode
    DMA1_Stream6->CR |= (0b10 << 16); // priority high
    DMA1_Stream6->CR |= (0x1 << 6);

    DMA1_Stream6->M0AR = (uint32_t)data;
    DMA1_Stream6->PAR = (uint32_t)&USART2->DR;
    DMA1_Stream6->NDTR = size;


    DMA1_Stream6->CR |= (1 << 0);
}

#define MSG_BUFFER_SIZE 128

void os_putchar(char c) {
//  static char msg_buffer[MSG_BUFFER_SIZE * 4]; //  static uint64_t last_time = 0;
//  static uint16_t msg_pointe = 0;
//  static uint16_t current_buffer = 0;
//  if (msg_pointer < MSG_BUFFER_SIZE && last_time + 10 * MILLISECONDS >= now()) {
//      msg_buffer[msg_pointer] = c;
//      msg_pointer++;
//  } else {
//      USART_put_dma(msg_buffer, msg_pointer);
//      msg_pointer = 0;
//      last_time = now();
//      msg_buffer[msg_pointer] = c;
//      msg_pointer++;
//      current_buffer
//  }
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

void os_putint(int num) {
    char * result = (char*) os_alloc(sizeof(char) * 16);
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

void os_putf(double num) {
          int i_part = num;
          double f_part = (num - i_part);
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

msg_object l = {.next = (msg_object*) nullptr, .msg = (char*) nullptr, .size = 0};
msg_object * last = &l;
msg_object * head = &l;
int msg_put(char *msg, size_t size) {
    msg_object * new_msg = (msg_object*) os_alloc(sizeof(msg_object));
    if (new_msg == (msg_object*) nullptr) {
        return 0;
    }
    head->next = new_msg;
    new_msg->next = (msg_object*) nullptr;
    new_msg->msg = msg;
    new_msg->size = size;
    head = new_msg;
    return 1;
}

int msg_send_next() {
    msg_object * current = last->next;
    if (current == (msg_object *) nullptr) return 0;
    if (current->msg != (char *)nullptr && current->size != 0) {
        USART_put_dma(current->msg, current->size);
    }
    if (last->msg != (char *)nullptr & os_test_mem(last->msg)) os_free(last->msg);
    if (last != (msg_object *)nullptr) os_free(last);
    last = current;
    return 1;
}

int dma_free() {
    if ((DMA1_Stream6->CR & (1 << 0)) == 0) { // Check for free 
        return 1;
    }
    return 0;
}

volatile void msg_thread() {
    while (1) {
      if (dma_free()) {
        int ret = msg_send_next();
        if (ret == 0)
          yield();
      } else {
        yield();
      }
    }
}
