//
// Created by Sarang Suman on 3/11/25.
//

#include "Quaternion.h"
#include <cmath>


Quaternion::Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

int main() {

}

// Conjugate of the quaternion
Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

// Addition of two quaternions
Quaternion Quaternion::add(const Quaternion& q) const {
    return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
}

// Multiplication of two quaternions
Quaternion Quaternion::multiply(const Quaternion& q) const {
    return Quaternion(
        w * q.w - x * q.x - y * q.y - z * q.z,
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w
    );
}

// Norm (magnitude) of the quaternion
double Quaternion::norm() const {
    return std::sqrt(w * w + x * x + y * y + z * z);
}

// Convert quaternion to rotation matrix
void Quaternion::toRotationMatrix(double matrix[3][3]) const {
    matrix[0][0] = 1 - 2 * (y * y + z * z);
    matrix[0][1] = 2 * (x * y - z * w);
    matrix[0][2] = 2 * (x * z + y * w);

    matrix[1][0] = 2 * (x * y + z * w);
    matrix[1][1] = 1 - 2 * (x * x + z * z);
    matrix[1][2] = 2 * (y * z - x * w);

    matrix[2][0] = 2 * (x * z - y * w);
    matrix[2][1] = 2 * (y * z + x * w);
    matrix[2][2] = 1 - 2 * (x * x + y * y);
}
