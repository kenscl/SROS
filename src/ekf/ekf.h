#ifndef __EKF
#define __EKF
#include "../math/matrix.h"
#include "../math/quaternion.h"
#define M_PI 3.14159265358979323846264338327950288419716939937510 
class EKF {
public:
    Quaternion attitude;
    Vec3 bias;
    Vec<10> x;
    Vec4 y, z;
    Mat<10, 10> P, F, Q;
    Mat<4, 10> H;
    Mat<10, 4> K;
    Mat<4,4> R;
    EKF();
    Mat3 Rot;
    Mat3 Rot_inv;
    void init(Vec3 * gyro, Vec3 * acc, Vec3 * mag);
    void update_acc(Vec3 acc);
    void update_mag(Vec3 mag);
    void predict(Vec3 gyro, double dt);
    void update();
};

#endif
