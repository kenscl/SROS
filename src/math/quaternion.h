#ifndef __QUATERNION
#define __QUATERNION
#include "matrix.h"
#include "vector.h"
#include <stddef.h>
#include <stdint.h>

typedef struct Quat {
    float q, i, j, k;
} Quat;

Quat *quat_alloc() {
    Quat *quat = (Quat *)os_alloc(sizeof(Quat));
    if (quat == 0)
        return 0;
    quat->q = 0.0f;
    quat->i = 0.0f;
    quat->j = 0.0f;
    quat->k = 0.0f;
    return quat;
}

int quat_from_rpy(Quat *quat, float roll, float pitch, float yaw) {
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);

    quat->q = cr * cp * cy + sr * sp * sy;
    quat->i = sr * cp * cy - cr * sp * sy;
    quat->j = cr * sp * cy + sr * cp * sy;
    quat->k = cr * cp * sy - sr * sp * cy;
    return 1;
}

int quat_from_vec3(Vec *rpy, Quat *quat) {
    float roll = rpy->r[0];
    float pitch = rpy->r[1];
    float yaw = rpy->r[2];

    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);

    quat->q = cr * cp * cy + sr * sp * sy;
    quat->i = sr * cp * cy - cr * sp * sy;
    quat->j = cr * sp * cy + sr * cp * sy;
    quat->k = cr * cp * sy - sr * sp * cy;
    return 1;
}

int quat_from_vec4(Vec *vec, Quat *quat) {
    quat->q = vec->r[0];
    quat->i = vec->r[1];
    quat->j = vec->r[2];
    quat->k = vec->r[3];
}

// Quaternion(const Mat3 &mat) {
//     Quaternion ret = Quaternion(1, 0, 0, 0);
//
//     float trace = mat[0][0] + mat[1][1] + mat[2][2];
//
//     if (trace > 0) {
//         float s = 0.5 / sqrt(trace + 1);
//         this->q = 0.25 / s;
//         this->i = (mat[2][1] - mat[1][2]) * s;
//         this->j = (mat[0][2] - mat[2][0]) * s;
//         this->k = (mat[1][0] - mat[0][1]) * s;
//     } else {
//         if (mat[0][0] > mat[1][1] && mat[0][0] > mat[2][2]) {
//             float s = 2 * sqrt(1 + mat[0][0] - mat[1][1] - mat[2][2]);
//             this->q = (mat[2][1] - mat[1][2]) / s;
//             this->i = 0.25 * s;
//             this->j = (mat[0][1] + mat[1][0]) / s;
//             this->k = (mat[0][2] + mat[2][0]) / s;
//         } else if (mat[1][1] > mat[2][2]) {
//             float s = 2 * sqrt(1 + mat[1][1] - mat[0][0] - mat[2][2]);
//             this->q = (mat[0][2] - mat[2][0]) / s;
//             this->i = (mat[0][1] + mat[1][0]) / s;
//             this->j = 0.25 * s;
//             this->k = (mat[1][2] + mat[2][1]) / s;
//         } else {
//             float s = 2 * sqrt(1 + mat[2][2] - mat[0][0] - mat[1][1]);
//             this->q = (mat[1][0] - mat[0][1]) / s;
//             this->i = (mat[0][2] + mat[2][0]) / s;
//             this->j = (mat[1][2] + mat[2][1]) / s;
//             this->k = 0.25 * s;
//         }
//     }
// }

int quat_mult(Quat *a, Quat *b, Quat *result) {
    result->q = a->q * b->q - a->i * b->i - a->j * b->j - a->k * b->k;
    result->i = a->q * b->i + a->i * b->q + a->j * b->k - a->k * b->j;
    result->j = a->q * b->j - a->i * b->k + a->j * b->q + a->k * b->i;
    result->k = a->q * b->k + a->i * b->j - a->j * b->i + a->k * b->q;
    return 1;
}

int quat_add(Quat *a, Quat *b, Quat *result) {
    result->q = a->q + b->q;
    result->i = a->i + b->i;
    result->j = a->j + b->j;
    result->k = a->k + b->k;
    return 1;
}

int quat_scalar_mult(Quat *a, float f) {
    a->q *= f;
    a->i *= f;
    a->j *= f;
    a->k *= f;
    return 1;
}

int quat_conjugate(Quat *a) {
    a->q = -a->q;
    a->i = -a->i;
    a->j = -a->j;
    a->k = -a->k;
}

int quat_to_rpy(Quat *a, Vec *result) {
    if (result->size != 3)
        return 0;
    result->r[0] = atan2(2 * (a->q * a->i + a->j * a->k), 1 - 2 * (a->i * a->i + a->j * a->j));
    result->r[1] = asin(2 * (a->q * a->j - a->k * a->i));
    result->r[2] = atan2(2 * (a->q * a->k + a->i * a->j), 1 - 2 * (a->j * a->j + a->k * a->k));
    return 1;
}

int quat_to_vector(Quat *a, Vec *ret) {
    ret->r[0] = a->q;
    ret->r[1] = a->i;
    ret->r[2] = a->j;
    ret->r[3] = a->k;
    return 1;
}

int quat_to_rotation_matrix(Quat *a, Mat* res)  {
  if (res->m != 3 || res->n != 3) return 0;
    res->[0 * res->n + 0] = 1 - 2 * (a->j * a->j + a->k * a->k);
    res->[0 * res->n + 1] = 2 * (a->i * a->j - a->k * a->q);
    res->[0 * res->n + 2] = 2 * (a->i * a->k + a->j * a->q);
    res->[1 * res->n + 0] = 2 * (a->i * a->j + a->k * a->q);
    res->[1 * res->n + 1] = 1 - 2 * (a->i * a->i + a->k * a->k);
    res->[1 * res->n + 2] = 2 * (a->j * a->k - a->i * a->q);
    res->[2 * res->n + 0] = 2 * (a->i * a->k - a->j * a->q);
    res->[2 * res->n + 1] = 2 * (a->j * a->k + a->i * a->q);
    res->[2 * res->n + 2] = 1 - 2 * (a->i * a->i + a->j * a->j);
    return 1;
}

float quat_norm(Quat *a) {
    return sqrtf(a->q * a->q + a->i * a->i + a->j * a->j + a->k * a->k);
}

int quat_normalize(Quat *a) {
    float norm = quat_norm(a);
    if (norm == 0)
        norm = 1;
    a->q = a->q / norm;
    a->i = a->i / norm;
    a->j = a->j / norm;
    a->k = a->k / norm;
    return 1;
}

void quat_print(Quat *a) {
    os_printf("Quaternion: [%f %f %f %f]", a->q, a->i, a->j, a->k);
}

#endif
