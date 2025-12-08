#pragma once

#include <stdint.h>

typedef uint64_t os_time_t;

#define DEBUG                 0

#define MAX_MATRIX            10

// os defines
#define OS_MAX_THREAD_COUNT   8
#define OS_STD_STACK_SIZE     200
#define STD_THREAD_PRIORITY   10
#define STD_THREAD_NAME       "Unnamed thread"
#define OS_ALLOC_HEAP_SIZE    1024 * 60

// SPI
#define SPI_buffer_lenght     20


#define MILLISECONDS 1
#define SECONDS 1000
#define MINUTES SECONDS * 60
#define HOURS MINUTES * 60
#define DAYS HOURS * 24

void OS_WARN (char * msg);
