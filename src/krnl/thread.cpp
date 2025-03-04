#include "thread.h"
#include "scheduler.h"
#include "../globals.h"
#include "mem.h"
#include <stdio.h>
#include <cstdlib>



os_pcb *register_thread_auto(volatile void (*thread_handler)()) {
    return register_thread_auto(thread_handler, OS_STD_STACK_SIZE, STD_THREAD_PRIORITY, STD_THREAD_NAME);
}

os_pcb *register_thread_auto(volatile void (*thread_handler)(), uint32_t stack_size, uint8_t priority, char* name) {
    uint32_t *stack = (uint32_t *) os_alloc(stack_size * sizeof(uint32_t));  
    if (stack == nullptr) {
      return 0;
    }
    os_pcb * thrd = (os_pcb *) os_alloc(sizeof(os_pcb));         
    if (thrd == nullptr) {
      return 0;
    }
    thrd->rdy = 1;
    thrd->priority = priority;
    thrd->name = name;
    thrd->last_time = 0;
    thrd->sleep_until = 0;
    os_stack_init(thrd, (void *) thread_handler, stack, stack_size);
    
    register_thread(thrd);

    return thrd;
}

void sleep(uint64_t time) {
  current_thread->sleep_until = now() + time;
  yield();
}
