#ifndef __EKF
#define __EKF
#include "../math/matrix.h"
#include "../math/quaternion.h"
class EKF {
public:
    Quaternion attitude;
    Vec3 bias;
    Vec<10> x;
    Vec4 y, z;
    Mat<10, 10> P, F, Q, K;
    Mat<4, 10> H;
    Mat<4,4> R;
    EKF();
    Mat3 Rot;
    Mat3 Rot_inv;
    void init(Vec3 * gyro, Vec3 * acc, Vec3 * mag);
    void update_acc(Vec3 acc);
    void update_mag(Vec3 mag);
    void predict(Vec3 gyro, double dt);
    void update();
}

#endif
