#include "mathhelper.h"
#include <cmath>

MathHelper::MathHelper() {}
MathHelper::~MathHelper() {}

Vector MathHelper::quaternions_to_euler_angular_velocities(const Quaternion& q1, const Quaternion& q2, double dt) {
    Quaternion dq = q2 * q1.inverse();
    Vector euler = quaternion_to_euler(dq);
    return euler * (1/dt);
}

Quaternion MathHelper::euler_to_quaternion(const Vector& euler) {
    double c1 = cos(euler[0] / 2);
    double s1 = sin(euler[0] / 2);
    double c2 = cos(euler[1] / 2);
    double s2 = sin(euler[1] / 2);
    double c3 = cos(euler[2] / 2);
    double s3 = sin(euler[2] / 2);

    return Quaternion(c1 * c2 * c3 + s1 * s2 * s3,
                      s1 * c2 * c3 - c1 * s2 * s3,
                      c1 * s2 * c3 + s1 * c2 * s3,
                      c1 * c2 * s3 - s1 * s2 * c3);
}

Vector MathHelper::quaternion_to_euler(const Quaternion& quaternion) {
    double* eulerAngles = new double[3] {
        atan2(2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]), (1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]))),
        asin(2 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1])),
        atan2(2 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]), (1 - 2 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3])))
    };
    return Vector(3, eulerAngles);
}

Matrix MathHelper::get_extrinsic_x_rotation(double x) {
    double* data = new double[9] {
        1, 0, 0, 
        0, cos(x), -sin(x), 
        0, sin(x), cos(x)
    };
    return Matrix(3, 3, data);
}

Matrix MathHelper::get_extrinsic_y_rotation(double y) {
    double* data = new double[9] {
        cos(y), 0, -sin(y), 
        0, 1, 0, 
        sin(y), 0, cos(y)
    };
    return Matrix(3, 3, data);
}

Matrix MathHelper::get_extrinsic_z_rotation(double z) {
    double* data = new double[9] {
        cos(z), -sin(z), 0, 
        sin(z), cos(z), 0, 
        0, 0, 1
    };
    return Matrix(3, 3, data);
}

Matrix MathHelper::get_extrinsic_XYZ_rotation_matrix(const Vector& euler) {
    return get_extrinsic_z_rotation(euler[2]) * get_extrinsic_y_rotation(euler[1]) * get_extrinsic_x_rotation(euler[0]);
}