#ifndef __VECTOR
#define __VECTOR
#include "../communication/usart.h"
#include "../krnl/mem.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>

typedef struct Vec {
    size_t size;
    float *r;
} Vec;

Vec *vec_alloc(size_t size);
void vec_free(Vec *a);
void vec_print(Vec *vec);
int vec_add(Vec *a, Vec *b, Vec *result);
int vec_sub(Vec *a, Vec *b, Vec *result);
int vec_mult(Vec *a, Vec *b, Vec *result);
int vec_dot(Vec *a, Vec *b, float *result);
int vec_scalar_mult(Vec *a, float f);
float vec_norm(Vec *vec);
int vec_normalize(Vec *a);
int vec_equals(Vec *a, Vec *b);

#endif
