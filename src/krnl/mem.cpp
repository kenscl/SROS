#include "mem.h"
#include <stdio.h>

struct Chunk{
    uint16_t size;
    uint8_t free;
} Chunk;

uint8_t heap[OS_ALLOC_HEAP_SIZE] __attribute__((aligned(8)));
#define LEDGER_SIZE    (OS_ALLOC_HEAP_SIZE + 63) / 64 + 1
struct Chunk ledger[LEDGER_SIZE];

uint8_t math_heap[MATH_ALLOC_HEAP_SIZE] __attribute__((aligned(8)));
#define MATH_LEDGER_SIZE    (MATH_ALLOC_HEAP_SIZE + 7) / 8+ 1
struct Chunk math_ledger[MATH_LEDGER_SIZE];

void mem_init() {
    for (int i = 0; i < LEDGER_SIZE - 1; ++i) {
        ledger[i].free = 1;
        ledger[i].size = 0;
    }
    ledger[LEDGER_SIZE - 1].free = 0;
    for (int i = 0; i < MATH_LEDGER_SIZE - 1; ++i) {
        math_ledger[i].free = 1;
        math_ledger[i].size = 0;
    }
    math_ledger[MATH_LEDGER_SIZE - 1].free = 0;
}
int alloc;
void *os_alloc(size_t size) {
    alloc = 1;
    size = (size + 63) & ~63;  // Align to the next multiple of 64 
    size = size / 64; // adjust to the ledger size
    uint16_t current_begin = 0;
    uint16_t current_size = 0;
    uint16_t best_fit = 65535;
    uint16_t best_fit_size = 65535;
    // find the best fit
    for (int i = 0; i < LEDGER_SIZE; ++i) {
        if (current_size == 0) {
            current_begin = i;
        }
        if (ledger[i].free) {
            current_size++;
        } else {
            if (current_size >= size && best_fit_size > current_size) {
                best_fit = current_begin;
                best_fit_size = current_size;
                current_size = 0;
            }
        }
    }

    if (best_fit_size == 65535) {
        return nullptr;// error return
    }

    // shrink best fit until its small enough
    while (best_fit_size / 2 >= size) {
        best_fit_size /= 2;
    }

    // allocate parts
    for (int i = best_fit; i < best_fit + best_fit_size; ++i) {
        ledger[i].free = 0;
        ledger[i].size = best_fit + best_fit_size - i;
    }

    // return allocated
    return (void *) &heap[best_fit * 64];
    alloc = 0;
}

void os_free(void * pointer) {
    int id = ((uint64_t) pointer - (uint64_t)&heap) / 64;
    struct Chunk * cur = &ledger[id];
    int size = cur->size;
    for  (int i = 0; i < size; i++) {
        cur->free = 1;
        cur->size = 0;
        cur = &ledger[++id];
    }
}

void *math_alloc(size_t size) {
    alloc = 1;
    size = (size + 7) & ~7;  // Align to the next multiple of 64 
    size = size / 8; // adjust to the math_ledger size
    uint16_t current_begin = 0;
    uint16_t current_size = 0;
    uint16_t best_fit = 65535;
    uint16_t best_fit_size = 65535;
    // find the best fit
    for (int i = 0; i < MATH_LEDGER_SIZE; ++i) {
        if (current_size == 0) {
            current_begin = i;
        }
        if (math_ledger[i].free) {
            current_size++;
        } else {
            if (current_size >= size && best_fit_size > current_size) {
                best_fit = current_begin;
                best_fit_size = current_size;
                current_size = 0;
            }
        }
    }

    if (best_fit_size == 65535) {
        OS_WARN("MATH IS LOW ON MEMORY!");
        return nullptr;// error return
    }

    // shrink best fit until its small enough
    while (best_fit_size / 2 >= size) {
        best_fit_size /= 2;
    }

    // allocate parts
    for (int i = best_fit; i < best_fit + best_fit_size; ++i) {
        math_ledger[i].free = 0;
        math_ledger[i].size = best_fit + best_fit_size - i;
    }

    // return allocated
    return (void *) &math_heap[best_fit * 8];
    alloc = 0;
}

void math_free(void * pointer) {
    int id = ((uint64_t) pointer - (uint64_t)&math_heap) / 8;
    struct Chunk * cur = &math_ledger[id];
    int size = cur->size;
    for  (int i = 0; i < size; i++) {
        cur->free = 1;
        cur->size = 0;
        cur = &math_ledger[++id];
    }
}
