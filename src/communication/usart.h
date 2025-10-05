#ifndef USART
#define USART
#include <stddef.h>
#include <stdint.h>
#include "../krnl/scheduler.h"

typedef struct msg_object {
    struct msg_object * next;
    char * msg;
    size_t size;
} msg_object;

void clock_init_usart1();
void enable_usart();

// actual print fucntions
//void os_putchar(char c);
void os_putstr(char *s);
//void os_putstr(char *s, size_t size);
void os_putint(int i);
void os_putf(float num);
void os_printf(char * format, ...);

// used for actual printing
int msg_put(char *msg, size_t size);


#endif
