#ifndef MEM
#define MEM
#include <stdint.h>
#include <stdio.h>
#include "../globals.h"

void mem_init();
void * os_alloc(size_t size);
void os_free(void * pointer);

#endif
