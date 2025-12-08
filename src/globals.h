#pragma once

#include <stdint.h>

typedef uint64_t os_time_t;

#define MILLISECONDS 1
#define SECONDS 1000
#define MINUTES SECONDS * 60
#define HOURS MINUTES * 60
#define DAYS HOURS * 24

void OS_WARN (char * msg);
