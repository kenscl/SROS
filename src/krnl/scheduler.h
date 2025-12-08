#ifndef scheduler
#define scheduler
#include "thread.h"
#include "../config.h"
#include "../globals.h"

extern uint64_t ticks;

extern os_pcb *volatile current_thread;
extern "C" {
void schedule();
}
extern uint8_t sched_on;

void scheduler_init();
void scheduler_enable();
void scheduler_disable();
int register_thread(os_pcb * thread);
int remove_thread(os_pcb * thread);
uint64_t now();
uint64_t now_high_accuracy(); // µs
void print_thread_info();

#endif
