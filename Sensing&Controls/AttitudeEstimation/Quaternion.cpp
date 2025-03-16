#include "Quaternion.h"
#include <cmath>

Quaternion::Quaternion() : data(new double[4] {0, 0, 0, 0}) {} //default constructor

Quaternion::Quaternion(double w, double x, double y, double z) : data(new double[4] {w, x, y, z}) {} //constructor with parameter

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


