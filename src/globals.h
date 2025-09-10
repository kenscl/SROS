#pragma once
#include <stdint.h>

typedef uint64_t os_time_t;

// os defines
#define OS_MAX_THREAD_COUNT   8
#define OS_STD_STACK_SIZE     200 
#define STD_THREAD_PRIORITY   10
#define STD_THREAD_NAME       "Unnamed thread"
#define OS_ALLOC_HEAP_SIZE    1024 * 10


#define MILLISECONDS 1
#define SECONDS 1000
#define MINUTES SECONDS * 60
#define HOURS MINUTES * 60
#define DAYS HOURS * 24

extern uint8_t forced_restard;

void OS_WARN (char * msg); 
void OS_PANIC (char* msg);


