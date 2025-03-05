#ifndef thread
#define thread
#include <stdint.h>
#include "../globals.h"


typedef struct {
  // if this ordering is changed the compiler will later insert ldrd instructions that cause a hard fault
		void* sp;
        char* name;
		os_time_t last_time;
		uint8_t priority;
		uint8_t rdy;
        uint64_t sleep_until;
} os_pcb;


void yield();
void sleep(uint64_t time);
void sleep_until(uint64_t time);

void os_stack_init(os_pcb * thread, void * thread_handler, void * stack, uint32_t size_of_stack);

/*
 * This is the function you want to call if you want to create a thread.
 */

#ifdef __cplusplus
os_pcb *register_thread_auto(volatile void (*thread_handler)());
os_pcb *register_thread_auto(volatile void (*thread_handler)(), uint32_t stack_size, uint8_t priority, char* name);
#define OS_THREAD(handler) register_thread_auto(&handler);
#endif

  
#endif
