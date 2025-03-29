//
// Created by Sarang Suman on 3/11/25.
//

#include "Quaternion.h"
#include <cmath>
#include <vector>
#include "Matrix.h"


Quaternion::Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}


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
Matrix Quaternion::toRotationMatrix() const {
    Matrix matrix(3, 3, 0);
    matrix(0, 0) = 1 - 2 * (y * y + z * z);
    matrix(0, 1) = 2 * (x * y - z * w);
    matrix(0, 2) = 2 * (x * z + y * w);

    matrix(1, 0) = 2 * (x * y + z * w);
    matrix(1, 1) = 1 - 2 * (x * x + z * z);
    matrix(1, 2) = 2 * (y * z - x * w);

    matrix(2, 0) = 2 * (x * z - y * w);
    matrix(2, 1) = 2 * (y * z + x * w);
    matrix(2, 2) = 1 - 2 * (x * x + y * y);

    return matrix;
}


// Convert quaternion to Euler angles (roll, pitch, yaw)
Matrix Quaternion::toEulerMatrix() const {
    Matrix eulerMatrix(3, 1, 0);

    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    eulerMatrix(0, 0) = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        eulerMatrix(1, 0) = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    else
        eulerMatrix(1, 0) = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    eulerMatrix(2, 0) = std::atan2(siny_cosp, cosy_cosp);

    return eulerMatrix;
}