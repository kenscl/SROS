#include "globals.h"
#include "communication/usart.h"
#include "krnl/scheduler.h"
#include "hal/hw_specific.h"
void OS_WARN (char * msg) {
    os_interrupt_disable();
    os_printf("WARNING: %s \n", msg);
    os_interrupt_enable();
}

uint8_t forced_restard = 0;
void OS_PANIC (char* msg) {
    os_interrupt_disable();
    scheduler_disable();
    os_printf("PANIC (hard error): %s \n", msg); 
    os_printf("Calling reset_handler! \n");
    forced_restard = 1;
    void (*reset_handler)(void) = (void (*)(void))(*((uint32_t*)0x00000004));
    reset_handler();
}
