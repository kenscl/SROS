#include "globals.h"
#include "usart.h"
#include "scheduler.h"
void OS_WARN (char * msg) {
    __disable_irq();
    os_printf("WARNING: %s \n", msg);
    __enable_irq();
}

uint8_t forced_restard = 0;
void OS_PANIC (char* msg) {
    __disable_irq();
    scheduler_disable();
    os_printf("PANIC (hard error): %s \n", msg); 
    os_printf("Calling reset_handler! \n");
    forced_restard = 1;
    void (*reset_handler)(void) = (void (*)(void))(*((uint32_t*)0x00000004));
    reset_handler();
}
