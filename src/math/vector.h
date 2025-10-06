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

Vec *vec_alloc(size_t size) {
    Vec *vec = (Vec *)os_alloc(sizeof(Vec));
    if (vec == 0)
        return 0;
    vec->size = size;
    vec->r = (float *)os_alloc(sizeof(float) * size);
    if (vec->r == 0)
        return 0;
    for (int i = 0; i < size; ++i) {
        vec->r[i] = 0.0f;
    }
    return vec;
}

void vec_free(Vec *a) {
    os_free(a->r);
    os_free(a);
}

void vec_print(Vec *vec) {
    os_printf("Vector: [");
    for (int i = 0; i < vec->size; ++i) {
	if (vec->r[i] != vec->r[i])
	    os_printf("NaN ");
	else
	    os_printf("%f ", vec->r[i]);
    }
    os_printf("] \n");
}

int vec_add(Vec *a, Vec *b, Vec *result) {
    if (a->size != b->size || a->size != result->size)
        return 0;
    for (int i = 0; i < a->size; ++i) {
        result->r[i] = a->r[i] + b->r[i];
    }
    return 1;
}

int vec_sub(Vec *a, Vec *b, Vec *result) {
    if (a->size != b->size || a->size != result->size)
        return 0;
    for (int i = 0; i < a->size; ++i) {
        result->r[i] = a->r[i] - b->r[i];
    }
    return 1;
}

int vec_mult(Vec *a, Vec *b, Vec *result) {
    if (a->size != b->size || a->size != result->size)
        return 0;
    for (int i = 0; i < a->size; ++i) {
        result->r[i] = a->r[i] * b->r[i];
    }
    return 1;
}

int vec_dot(Vec *a, Vec *b, float *result) {
    if (a->size != b->size)
        return 0;
    *result = 0;
    for (int i = 0; i < a->size; ++i) {
        *result += a->r[i] * b->r[i];
    }
    return 1;
}

int vec_scalar_mult(Vec *a, float f) {
    for (int i = 0; i < a->size; ++i) {
        a->r[i] = a->r[i] * f;
    }
    return 1;
}

float vec_norm(Vec *vec) {
    float res = 0;
    for (int i = 0; i < vec->size; ++i) {
        float val = vec->r[i];
        res += val * val;
    }
    return sqrtf(res);
}

int vec_normalize(Vec *a) {
    float norm = vec_norm(a);
    norm = 1 / norm;
    vec_scalar_mult(a, norm);
    return 1;
}

int vec_equals(Vec *a, Vec *b) {
  if (a->size != b->size) return 0;
  for (int i = 0; i < a->size; ++i) {
    if (a->r[i] != b->r[i]) return 0;
  }
  return 1;
}
#endif
