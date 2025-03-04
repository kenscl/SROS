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
#ifdef __cplusplus
void os_putstr(char *s);
void os_putstr(char *s, size_t size);
void os_putint(int i);
void os_putf(float num);
#endif

// used for actual printing
int msg_put(char *msg, size_t size);
/*
 * msg_send_next frees the last msg and msg_object and puts the new message into the dma buffer (i know, thats not how it works :))
 * returns 0 if the next message is invalid, otherwise returns 1
 */
int msg_send_next();

int dma_free();

volatile void msg_thread();


#endif
