#include "scheduler.h"
#include <stm32f1xx.h>
#include "usart.h"
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

        current_thread->last_time = schedule_counter;
  
        for (uint16_t i = 0; i < OS_MAX_THREAD_COUNT; ++i){
            uint8_t exists = thread_list[i]->sp != 0;
            uint8_t is_ready = thread_list[i]->rdy == 1;
            uint8_t is_not_sleeping = thread_list[i]->sleep_until <= now();

            if ( exists && is_ready && is_not_sleeping ) {
                uint8_t is_larger_priority = thread_list[i]->priority >= current_thread->priority;
                uint8_t last_time_was_earlier = thread_list[i]->last_time < current_thread->last_time;
                uint8_t current_is_sleeping = current_thread->sleep_until >= now();
                if ( (is_larger_priority || current_is_sleeping ) && last_time_was_earlier ) {
                    current_thread = thread_list[i];
                }
            }
        }
        ++schedule_counter;
    }

  __attribute__((naked)) void pendsv_handler(void) {

    __asm volatile (
                    "CPSID         I \n"

                    "LDR           r1,=current_thread \n"
                    "LDR           r1,[r1,#0x00] \n"
                    "CBZ           r1,_restore \n"

                    "stmdb	sp!, {r4, r5, r6, r7, r8, r9, r10, r11, r14} \n"

                    "LDR           r1,=current_thread \n"
                    "LDR           r1,[r1,#0x00] \n"
                    "STR           sp,[r1,#0x00] \n"

                    "_restore: \n"
                    "bl schedule \n"
                    "LDR           r1,=current_thread \n"
                    "LDR           r1,[r1,#0x00] \n"
                    "LDR           sp,[r1,#0x00] \n"

                    "ldmia	sp!, {r4, r5, r6, r7, r8, r9, r10, r11, r14} \n"

                    "CPSIE         I \n"

                    "BX            lr \n");
  }

  void systick_handler()
  {
    ticks++;
    if (!sched_on) return;
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
  }
}

uint64_t now() {
  return ticks;
}

void print_thread_info() {
    os_printf("Thread overview: \n");
    uint8_t cnt = 0;
    for (int i = 0; i < OS_MAX_THREAD_COUNT; ++i) {
        if (thread_list[i]->sp != 0) {
            cnt++;
            os_pcb * cur = thread_list[i];
            os_printf("Thread:      %s\n", cur->name);
            os_printf("Priority:    %d\n", (int) cur->priority);
            os_printf("Last time:   %d\n", (int) cur->last_time);
            os_printf("Ready:       %d\n", (int) cur->rdy);
            os_printf("Sleep until: %d\n", (int) cur->sleep_until);
            os_printf("\n");
        }
    }
    os_printf("In total there are %d threads.\n", cnt);
}