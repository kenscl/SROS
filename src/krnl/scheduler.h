#ifndef scheduler
#define scheduler
#include "thread.h"
#include "../globals.h"

extern uint64_t ticks;

#ifdef __cplusplus
extern "C" {
    extern os_pcb * volatile current_thread; 
    void schedule();
}
#endif
extern uint8_t sched_on;

void scheduler_init();
void scheduler_enable();
void scheduler_disable();
int register_thread(os_pcb * thread);
int remove_thread(os_pcb * thread);
uint64_t now();
uint32_t now_high_accuracy(); // µs
void print_thread_info();

#endif
