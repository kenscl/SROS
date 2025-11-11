#include "matrix.h"

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


void zero_mat(Mat *a) {
    for (int i = 0; i < a->m * a->n; i++) {
        a->r[i] = 0.0f;
    }
}

int mat_mult(Mat *a, Mat *b, Mat *res) {
    if (a->n != b->m)
        return 0;
    if (res->m != a->m || res->n != b->n)
        return 0;
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
    if (a->m != res->size)
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

MAT_ALLOC_STATIC(det_m, MAX_MATRIX, MAX_MATRIX)
float mat_det(Mat *a) {
    if (a->m != a->n) return 0;
    det_m.m = MAX_MATRIX;
    det_m.n = MAX_MATRIX;
    mat_fill(&det_m, 0);
    det_m.m = a->m;
    det_m.n = a->n;
    mat_copy(a, &det_m);
    float det = 1;

    for (int i = 0; i < a->n; ++i) {
        // Partial pivoting
        float max_elem = det_m.r[i * det_m.n + i];
        int max_row = i;
        for (int k = i + 1; k < a->n; ++k) {
            if (fabs(det_m.r[k * det_m.n + i]) > fabs(max_elem)) {
                max_elem = det_m.r[k * det_m.n + i];
                max_row = k;
            }
        }

        if (fabs(max_elem) < 1e-8) {  // singular
            return 0;
        }

        if (max_row != i) {
            for (int j = 0; j < a->n; ++j) {
                float tmp = det_m.r[i * det_m.n + j];
                det_m.r[i * det_m.n + j] = det_m.r[max_row * det_m.n + j];
                det_m.r[max_row * det_m.n + j] = tmp;
            }
            det *= -1;
        }

        for (int k = i + 1; k < a->n; ++k) {
            float factor = 0;
            if (fabs(det_m.r[i * det_m.n + i]) > 1e-8)
                factor = det_m.r[k * det_m.n + i] / det_m.r[i * det_m.n + i];
            else
                factor = 1;
            for (int j = i; j < a->n; ++j) {
                det_m.r[k * det_m.n + j] -= factor * det_m.r[i * det_m.n + j];
            }
        }

        det *= det_m.r[i * det_m.n + i];
    }

    return det;
}

MAT_ALLOC_STATIC(minor, MAX_MATRIX, MAX_MATRIX);
float mat_coaf(Mat *a, size_t i, size_t j) {
    minor.m = MAX_MATRIX;
    minor.n = MAX_MATRIX;
    mat_fill(&minor, 0);
    minor.m = a->m-1;
    minor.n = a->n-1;
    //Mat *minor = mat_alloc(a->m - 1, a->n - 1);
    int minor_row = 0, minor_col = 0;
    float sign = ((i + j) % 2 == 0) ? 1 : -1;

    for (int row = 0; row < a->m; ++row) {
        if (row == i)
            continue;
        minor_col = 0;
        for (int col = 0; col < a->n; ++col) {
            if (col == j)
                continue;
            minor.r[minor_row * minor.n + minor_col] = a->r[row * a->n + col];
            minor_col++;
        }
        minor_row++;
    }
    float res = sign * mat_det(&minor);
    return res;
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

int mat_inverse(Mat *a, Mat *res) {
    float det = mat_det(a);
    if (a->n != a->m || det == 0) {
        return 0;
    }
    if(!mat_adjugate(a, res)) return -1;
    if(!mat_scalar_mult(res, 1/det)) return -2;
    return 1;
}

int mat_identity(Mat *ident) {
    mat_fill(ident, 0);
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

int mat_copy(Mat *source, Mat *target) {
    if (source->m != target->n || source->n != target->n) {
        return 0;
    }

    for (int i = 0; i < source->m * source->n; i++) {
        target->r[i] = source->r[i];
    }
    return 1;
}

void mat_fill(Mat *m, float a) {
    for (int i = 0; i < m->m * m->n; i++) {
        m->r[i] = a;
    }
}
