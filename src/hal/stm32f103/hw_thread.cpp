#include "../../krnl/thread.h"
#include "stm32f1xx.h"
void os_stack_init(os_pcb * thrd, void * thread_handler, void * stack, uint32_t size_of_stack) {
    uint32_t* sp = (uint32_t*) ((((uint32_t)stack + size_of_stack)/8) * 8);
    *(--sp) = (1U << 24);                 // xPSR
    *(--sp) = (uint32_t)thread_handler;   //PC
    *(--sp) = 0x0000000EU;                //LR
    *(--sp) = 0x0000000CU;                //r12
    *(--sp) = 0x00000003U;                //r3 
    *(--sp) = 0x00000002U;                //r2
    *(--sp) = 0x00000001U;                //r1
    *(--sp) = 0x00000000U;                //r0

    *(--sp) = 0xfffffff9U; // LR - on initial run
    *(--sp) = 0x0000000BU; // r11-r4 
    *(--sp) = 0x0000000AU;
    *(--sp) = 0x00000009U; 
    *(--sp) = 0x00000008U; 
    *(--sp) = 0x00000007U; 
    *(--sp) = 0x00000006U; 
    *(--sp) = 0x00000005U; 
    *(--sp) = 0x00000004U; 

    stack = sp;
    thrd->sp = sp;
}

void yield() {
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}
