#ifndef MEM
#define MEM
#include <stdint.h>
#include <stdio.h>
#include "../globals.h"

extern int mem_full;
void mem_init();
void * os_alloc(size_t size);
void os_free(void * pointer);

/*
 * Test weather or not pointer was allocated with os_alloc 
 */
int os_test_mem(void * pointer);

#endif
