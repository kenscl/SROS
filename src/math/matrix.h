#ifndef __MATRIX
#define __MATRIX
#include "vector.h"
#include <stddef.h>
#include <stdint.h>

typedef struct Mat {
    size_t m, n;
    float *r;
} Mat;

Mat *mat_alloc(size_t m, size_t n);
void mat_free(Mat *a);
int mat_mult(Mat *a, Mat *b, Mat *res);
int mat_add(Mat *a, Mat *b, Mat *res);
int mat_sub(Mat *a, Mat *b, Mat *res);
int mat_scalar_mult(Mat *a, float f);
int mat_vec_mult(Mat *a, Vec *b, Vec *res);
float mat_det(Mat *a);
float mat_coaf(Mat *a, size_t i, size_t j);
int mat_adjugate(Mat *a, Mat *res);
int mat_transpose(Mat *a, Mat *res);
int mat_inverse(Mat *a, Mat *res);
int mat_identity(Mat *ident);
int mat_diag(Mat* diag, float d);
int mat_vec_diag(Mat *diag, Vec *v);
void mat_print(Mat *a);

#endif
