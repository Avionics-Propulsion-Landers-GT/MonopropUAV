#include "Vector.h"
#include <cmath>
#include "Quaternion.h"

double Vector::dotProduct(const Vector&other) const {
    if (this -> rows != other.rows) {
        return -0.01;
    }
    double result = 0.0;
    for (unsigned int i = 0; i < this-> rows; i++) {
        result += ((*this)(i, 0) * other(i,0));
    }
    return result;
}

double Vector::magnitude() const {
    double sum = 0.0;
    for (unsigned int i = 0; i < this-> rows; i++) {
        sum += ((*this)(i,0) * (*this)(i,0));
    }
    return sqrt(sum);
    //return (this->sqrt(sum));

}

double Vector::squareRoot(unsigned int num) const { 
    if (num < 0) {
        return -1.0;  // Error code
    }

    double guess = num / 2.0;
    double tolerance = 0.000000001;  // Accuracy of the result

    while (true) {
        double betterGuess = (guess + num / guess) / 2.0;
        if (abs(betterGuess - guess) < tolerance) {
            break;
        }
        guess = betterGuess;
    }

    return guess;
}

double Vector::size() const {
    return this->rows;
}

Vector Vector::normalize() const {
    double length = this->magnitude();
    if (length == 0) {
        return Vector(0, 0.0);
    }
    Matrix v = (*this) * (1.0/length);
    return Vector(v);
}

Vector Vector::crossProduct(const Vector&other) const {
    if (this-> getRows() != 3 && other.getRows() != 3) {
        return Vector(0,0.0);
    } else {
        Vector result = Vector(3, 0.0);
        result(0,0) = (*this)(1,0) * other(2,0) - (*this)(2,0) * other(1,0);
        result(1,0) = (*this)(2,0) * other(0,0) - (*this)(0,0) * other(2,0);
        result(2,0) = (*this)(0,0) * other(1,0) - (*this)(1,0) * other(0,0);
        return result;
    }

}

Quaternion Vector::toQuaternion() const {
    if (this->rows != 3) {
        throw std::invalid_argument("Vector must have 3 elements to convert to Quaternion");
    }
    return Quaternion(0.0, (*this)(0, 0), (*this)(1, 0), (*this)(2, 0));
}