#include "globals.h"
#include "communication/usart.h"
#include "krnl/scheduler.h"
#include "hal/hw_specific.h"
void OS_WARN (char * msg) {
}

uint8_t forced_restard = 0;
void OS_PANIC (char* msg) {
}
