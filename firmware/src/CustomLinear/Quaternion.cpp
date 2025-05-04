#include "Quaternion.h"
#include <cmath>
#include <vector>
#include "Matrix.h"

Quaternion::Quaternion() : data(new double[4] {0, 0, 0, 0}) {} //default constructor

Quaternion::Quaternion(double w, double x, double y, double z) : data(new double[4] {w, x, y, z}) {} //constructor with parameter

// --- Deep copy constructor ---
Quaternion::Quaternion(const Quaternion &other)
    : data(new double[4])
{
    for (unsigned int i = 0; i < 4; i++) {
        data[i] = other.data[i];
    }
}

// --- Deep copy assignment operator ---
Quaternion& Quaternion::operator=(const Quaternion &other) {
    if (this == &other) 
        return *this;
    delete[] data;
    data = new double[4];
    for (unsigned int i = 0; i < 4; i++) {
        data[i] = other.data[i];
    }
    return *this;
}

//destructor
Quaternion::~Quaternion() {
    delete[] data;
}

//access quaternion data
double& Quaternion::operator[](unsigned int index) {
    if (index >= 4) {
        throw std::out_of_range("Index out of range");
    }
    return data[index];
}

//access quaternion data
const double& Quaternion::operator[](unsigned int index) const{
    if (index >= 4) {
        throw std::out_of_range("Index out of range");
    }
    return data[index];
}

//quaternion addition
Quaternion Quaternion::operator+(const Quaternion& other) const {
    Quaternion result = Quaternion(this->data[0] + other[0], this->data[1] + other[1], this->data[2] + other[2], this->data[3] + other[3]);
    return result;
}

//quaternion subtraction
Quaternion Quaternion::operator-(const Quaternion& other) const {
    Quaternion result = Quaternion(this->data[0] - other[0], this->data[1] - other[1], this->data[2] - other[2], this->data[3] - other[3]);
    return result;
}

//quaternion multiplication by scalar
Quaternion Quaternion::operator*(double scalar) const{
    Quaternion result = Quaternion(this->data[0] * scalar, this->data[1] * scalar, this->data[2] * scalar, this->data[3] * scalar);
    return result;
}

//quaternion division
Quaternion Quaternion::operator/(double scalar) const{
    if (scalar == 0) {
        throw std::invalid_argument("Division by zero");
    }
    return *this * (1/scalar);
}

//quaternion multiplication by another quaternion
Quaternion Quaternion::operator*(const Quaternion& other) const{
    Quaternion result = Quaternion(this->data[0] * other[0] - this->data[1] * other[1] - this->data[2] * other[2] - this->data[3] * other[3],
                                    this->data[0] * other[1] + this->data[1] * other[0] + this->data[2] * other[3] - this->data[3] * other[2],
                                    this->data[0] * other[2] - this->data[1] * other[3] + this->data[2] * other[0] + this->data[3] * other[1],
                                    this->data[0] * other[3] + this->data[1] * other[2] - this->data[2] * other[1] + this->data[3] * other[0]);
    return result;
}

//quaternion negation
Quaternion Quaternion::operator-() const {
    Quaternion result = Quaternion(-this->data[0], -this->data[1], -this->data[2], -this->data[3]);
    return result;
}

//quaternion conjugate
Quaternion Quaternion::conjugate() const {
    Quaternion result = Quaternion(this->data[0], -this->data[1], -this->data[2], -this->data[3]);
    return result;
}

//find magnitude of quaternion
double Quaternion::magnitude() const {
    return sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2] + data[3] * data[3]);
}

//normalize quaternion
Quaternion Quaternion::normalize() const {
    double mag = this->magnitude();
    if (mag == 0) {
        return Quaternion(0, 0, 0, 0);
    } else {
        return *this / mag;
    }
}

//find inverse of quaternion
Quaternion Quaternion::inverse() const {
    double mag = this->magnitude();
    if (mag > 0) {
        return Quaternion(data[0], -data[1], -data[2], -data[3]) / mag;
    }
    return Quaternion(1, 0, 0, 0);
}

//rotate quaternion q by this quaternion
Quaternion Quaternion::rotate(const Quaternion& q) const {
    return *this * q * this->inverse();
}

// Convert quaternion to rotation matrix
Matrix Quaternion::toRotationMatrix() const {
    Matrix matrix(3, 3, 0);
    matrix(0, 0) = 1 - 2 * (data[2] * data[2] + data[3] * data[3]);
    matrix(0, 1) = 2 * (data[1] * data[2] - data[3] * data[0]);
    matrix(0, 2) = 2 * (data[1] * data[3] + data[2] * data[0]);

    matrix(1, 0) = 2 * (data[1] * data[2] + data[3] * data[0]);
    matrix(1, 1) = 1 - 2 * (data[1] * data[1] + data[3] * data[3]);
    matrix(1, 2) = 2 * (data[2] * data[3] - data[1] * data[0]);

    matrix(2, 0) = 2 * (data[1] * data[3] - data[2] * data[0]);
    matrix(2, 1) = 2 * (data[2] * data[3] + data[1] * data[0]);
    matrix(2, 2) = 1 - 2 * (data[1] * data[1] + data[2] * data[2]);

    return matrix;
}


// Convert quaternion to Euler angles (roll, pitch, yaw)
Matrix Quaternion::toEulerMatrix() const {
    Matrix eulerMatrix(3, 1, 0);

    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (data[0] * data[1] + data[2] * data[3]);
    double cosr_cosp = 1 - 2 * (data[1] * data[1] + data[2] * data[2]);
    eulerMatrix(0, 0) = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (data[0] * data[2] - data[3] * data[1]);
    if (std::abs(sinp) >= 1)
        eulerMatrix(1, 0) = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    else
        eulerMatrix(1, 0) = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (data[0] * data[3] + data[1] * data[2]);
    double cosy_cosp = 1 - 2 * (data[2] * data[2] + data[3] * data[3]);
    eulerMatrix(2, 0) = std::atan2(siny_cosp, cosy_cosp);

    return eulerMatrix;
}
