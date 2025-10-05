#ifndef __MATRIX
#define __MATRIX
#include "vector.h"
#include <stddef.h>
#include <stdint.h>

typedef struct Mat {
    size_t m, n;
    float *r;
} Mat;

Mat *mat_alloc(size_t m, size_t n) {
    Mat *mat = (Mat *)os_alloc(sizeof(Mat));
    if (mat == 0)
        return 0;
    mat->m = m;
    mat->n = n;
    mat->r = (float *)os_alloc(sizeof(float) * m * n);
    if (mat->r == 0)
        return 0;
    for (size_t i = 0; i < m; i++) {
        for (size_t j = 0; j < n; j++) {
            mat->r[i * mat->n + j] = 0.0f;
        }
    }
    return mat;
}

void mat_free(Mat *a) {
    os_free(a->r);
    os_free(a);
}

int mat_mult(Mat *a, Mat *b, Mat *res) {
    if (a->m != b->n)
        return 0;
    if (res->m != a->m || res->n != b->n)
        return -1;
    for (size_t i = 0; i < a->m; ++i) {
        for (size_t j = 0; j < b->n; ++j) {
            res->r[i * res->n + j] = 0.0f;
            for (size_t k = 0; k < a->n; ++k) {
                res->r[i * res->n + j] += a->r[i * a->n + k] * b->r[k * b->n + j];
            }
        }
    }
    return 1;
}

int mat_add(Mat *a, Mat *b, Mat *res) {
    if (a->m != b->m || a->n != b->n)
        return 0;
    if (res->m != a->m || res->n != a->n)
        return 0;

    int size = a->m * a->n;
    for (int i = 0; i < size; ++i) {
        res->r[i] = a->r[i] + b->r[i];
    }
    return 1;
}

int mat_sub(Mat *a, Mat *b, Mat *res) {
    if (a->m != b->m || a->n != b->n)
        return 0;
    if (res->m != a->m || res->n != a->n)
        return 0;

    int size = a->m * a->n;
    for (int i = 0; i < size; ++i) {
        res->r[i] = a->r[i] - b->r[i];
    }
    return 1;
}

int mat_scalar_mult(Mat *a, float f) {
    int size = a->m * a->n;
    for (int i = 0; i < size; ++i) {
        a->r[i] *= f;
    }
    return 1;
}

int mat_vec_mult(Mat *a, Vec *b, Vec *res) {
    if (a->n != b->size)
        return 0;
    if (b->size != res->size)
        return 0;

    for (size_t i = 0; i < a->m; ++i) {
        float sum = 0.0;
        for (size_t j = 0; j < a->n; ++j) {
            sum += a->r[i * a->n + j] * b->r[j];
        }
        res->r[i] = sum;
    }
    return 1;
}

float mat_det(Mat *a) {
    if (a->m != a->n) {
	os_printf("Requesting determinant of non-square Matrix!");
	return 0.0;
    }

    if (a->n == 1)
        return a->r[0];
    Mat *temp = mat_alloc(a->m, a->n);
    float det = 1;

    for (int i = 0; i < a->n; ++i) {
        float max_elem = temp->r[i * temp->n + i];
        int max_row = i;
        for (int k = i + 1; k < a->n; ++k) {
            if (fabs(temp->r[k * temp->n + i]) > fabs(max_elem)) {
                max_elem = temp->r[k * temp->n + i];
                max_row = k;
            }
        }

        if (max_row != i) {
            for (int j = 0; j < a->n; ++j) {
                float temp_val = temp->r[i * temp->n + j];
                temp->r[i * temp->n + j] = temp->r[max_row * temp->n + j];
                temp->r[max_row * temp->n + j] = temp_val;
            }
            det *= -1;
        }
        if (temp->r[i * temp->n + i] == 0) {
            return 0;
        }

        for (int k = i + 1; k < a->n; ++k) {
            float factor = temp->r[k * temp->n + i] / temp->r[i * temp->n + i];
            for (int j = i; j < a->n; ++j) {
                temp->r[k * temp->n + j] -= factor * temp->r[i * temp->n + j];
            }
        }

        det *= temp->r[i * temp->n + i];
    }

    mat_free(temp);
    return det;
}

float mat_coaf(Mat *a, size_t i, size_t j) {
    Mat *minor = mat_alloc(a->m - 1, a->n - 1);
    int minor_row = 0, minor_col = 0;
    float sign = ((i + j) % 2 == 0) ? 1 : -1;

    for (int row = 0; row < a->m; ++row) {
        if (row == i)
            continue;
        minor_col = 0;
        for (int col = 0; col < a->n; ++col) {
            if (col == j)
                continue;
            minor->r[minor_row * minor->n + minor_col] = a->r[row * a->n + col];
            minor_col++;
        }
        minor_row++;
    }
    return sign * mat_det(minor);
}

int mat_adjugate(Mat *a, Mat *res) {
    if (a->m != a->n) {
        return 0;
    }

    if (a->m != res->n || a->n != res->n) {
        return 0;
    }

    if (a->n == 1) {
        res->r[0] = 1;
        return 1;
    }

    for (int i = 0; i < a->n; ++i) {
        for (int j = 0; j < a->n; ++j) {
	  float cofactor = mat_coaf(a, i, j);
	  res->r[j * res->n + i] = cofactor;
        }
    }
    return 1;
}

int mat_transpose(Mat *a, Mat *res) {
  if (a->m != res->n || a->n != res->m) return 0;
    for (size_t i = 0; i < a->m; ++i) {
        for (size_t j = 0; j < a->n; ++j) {
            res->r[j * res->n + i] = a->r[i * a->n +j];
        }
    }
    return 1;
}

int inverse(Mat *a, Mat *res) {
    float det = mat_det(a);
    if (a->n != a->m || det == 0) {
        return 0;
    }
    if(!mat_adjugate(a, res)) return 0;
    if(!mat_scalar_mult(res, 1/det)) return 0;
    return 1;
}

int mat_identity(Mat *ident) {
    for (int i = 0; i < ident->n; ++i) {
        ident->r[i * ident->n +i] = 1;
    }
    return 1;
}

int mat_diag(Mat* diag, float d) {
    if (diag->n != diag->m)
      return 0;
    for (int i = 0; i < diag->n; ++i) {
        diag->r[i * diag->n +i] = d;
    }
    return 1;
}

int mat_vec_diag(Mat *diag, Vec *v) {
    if (diag->n != diag->m)
      return 0;
    if (diag->n != v->size) return 0;
    for (int i = 0; i < diag->n; ++i) {
        diag->r[i * diag->n +i] = v->r[i];
    }
    return 1;
}

void mat_print(Mat *a) {
    os_printf("Mat: \n");
    for (int i = 0; i < a->m; ++i) {
	os_printf("[ ");
	for (int j = 0; j < a->n; j++) {
	    if (a->r[i * a->n + j] != a->r[i * a->n + j])
		os_printf("NaN \n");
	    else
	      os_printf("%f ", a->r[i * a->n + j]);
	}
	os_printf("]\n");
    }
}

#endif
