#include "scheduler.h"
#include "../communication/usart.h"
#include "thread.h"
#include <cstdio>


uint64_t ticks = 0;
os_pcb pcb_pool[OS_MAX_THREAD_COUNT];
os_pcb * thread_list[OS_MAX_THREAD_COUNT];
uint8_t sched_on = 0;
void scheduler_enable()  {
    sched_on = 1;
}

void scheduler_disable() {
    sched_on = 0;
}
extern "C" {
  os_pcb * volatile current_thread; 
}
void scheduler_init() {
  for (uint16_t i = 0; i < OS_MAX_THREAD_COUNT; ++i) {
    thread_list[i] = &pcb_pool[i];
    thread_list[i]->sp = 0;
    current_thread = 0;
  }
}

int register_thread(os_pcb * thrd) {
  for (uint16_t i = 0; i < OS_MAX_THREAD_COUNT; ++i) {
    if (thread_list[i]->sp == 0) {
      thread_list[i] = thrd;
      return 0;
    }
  }
  return 1;
}

int remove_thread(os_pcb * thrd) {
  for (uint16_t i = 0; i < OS_MAX_THREAD_COUNT; ++i) {
    if (thread_list[i]->sp == thrd->sp) {
      thread_list[i]->sp = 0;
      return 0;
    }
  }
  return 1;
}



extern "C" {
    void schedule() {
        static uint64_t schedule_counter = 0;
        if (current_thread == 0) 
        {
            current_thread = thread_list[0];
            ++schedule_counter;
            return;
        } 

        current_thread->last_time = now_high_accuracy();
        volatile os_pcb * next_thread = current_thread;
  
        for (uint16_t i = 0; i < OS_MAX_THREAD_COUNT; ++i){
            uint8_t exists = thread_list[i]->sp != 0;
            uint8_t is_ready = thread_list[i]->rdy == 1;
            uint8_t is_not_sleeping = thread_list[i]->sleep_until <= now();

            if ( exists && is_ready && is_not_sleeping ) {
                uint8_t is_larger_priority = thread_list[i]->priority >= current_thread->priority;
                uint8_t last_time_was_earlier = thread_list[i]->last_time < current_thread->last_time;
                uint8_t current_is_sleeping = (current_thread->sleep_until >= now());
                if ( (is_larger_priority || current_is_sleeping ) && last_time_was_earlier ) {
                    next_thread = thread_list[i];
                }
            }
        }
        ++schedule_counter;
        current_thread = (os_pcb *) next_thread;
    }
}

uint64_t now() {
  return ticks;
}


void print_thread_info() {
    os_putstr("Thread overview: \n");
    uint8_t cnt = 0;
    for (int i = 0; i < OS_MAX_THREAD_COUNT; ++i) {
        if (thread_list[i]->sp != 0) {
            cnt++;
            os_pcb * cur = thread_list[i];
            os_putstr("Thread:      ");
            os_putstr(cur->name);
            os_putstr("\n", 1);
            os_putstr("Priority:    ");
            os_putint((int) cur->priority);
            os_putstr("\n", 1);
            os_putstr("Last time:   ");
            os_putint((int) cur->last_time);
            os_putstr("\n", 1);
            os_putstr("Ready:       ");
            os_putint((int) cur->rdy);
            os_putstr("\n");
            os_putstr("Sleep until: ");
            os_putint((int) cur->sleep_until);
            os_putstr("\n");
            os_putstr("\n");
        }
    }
    os_putstr("In total there are ");
    os_putint(cnt);
    os_putstr(" threads.\n");
}
