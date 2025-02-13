#ifndef __hw_specific
#define __hw_specific

#include "../krnl/scheduler.h"

void os_interrupt_enable();
void os_interrupt_disable();
void miscellaneous_init();
void idle_thread();
void print_welcome_msg();

#endif
