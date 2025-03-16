#ifndef MATH_HELPER_H
#define MATH_HELPER_H

#include "Matrix.h"
#include "Quaternion.h"
#include "Vector.h"

class MathHelper {
    public:
        MathHelper();
        ~MathHelper();
        Vector quaternions_to_euler_angular_velocities(const Quaternion& q1, const Quaternion& q2, double dt);
        Quaternion euler_to_quaternion(const Vector& euler);
        Vector quaternion_to_euler(const Quaternion& quaternion);
        Matrix get_extrinsic_x_rotation(double x);
        Matrix get_extrinsic_y_rotation(double y);
        Matrix get_extrinsic_z_rotation(double z);
        Matrix get_extrinsic_XYZ_rotation_matrix(const Vector& euler);
};
#endif