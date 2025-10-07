#ifndef __QUATERNION
#define __QUATERNION
#include "matrix.h"
#include "vector.h"
#include <stddef.h>
#include <stdint.h>

typedef struct Quat {
    float q, i, j, k;
} Quat;

Quat *quat_alloc();
void *quat_free(Quat *q);
int quat_from_rpy(Quat *quat, float roll, float pitch, float yaw);
int quat_from_vec3(Vec *rpy, Quat *quat);
int quat_from_vec4(Vec *vec, Quat *quat);
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

int quat_mult(Quat *a, Quat *b, Quat *result);
int quat_add(Quat *a, Quat *b, Quat *result);
int quat_scalar_mult(Quat *a, float f);
int quat_conjugate(Quat *a);
int quat_to_rpy(Quat *a, Vec *result);
int quat_to_vector(Quat *a, Vec *ret);
int quat_to_rotation_matrix(Quat *a, Mat* res);
float quat_norm(Quat *a);
int quat_normalize(Quat *a);
void quat_print(Quat *a);

#endif
