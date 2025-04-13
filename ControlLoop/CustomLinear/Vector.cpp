#include "Vector.h"
#include <iostream>

// Vector::Vector(unsigned int rows, double initVal)
//     : Matrix (rows, 1, initVal) {
//     if (rows > 0) {
//         data = new double[rows];
//         for (unsigned int i = 0; i < rows; ++i) {
//             data[i] = initVal;
//         }
//     }
//     }

// Vector::Vector(const Vector &other)
//     : Matrix(other.getRows()), data(nullptr)
// {
//     if (rows > 0) {
//         data = new double[rows];
//         for (unsigned int i = 0; i < rows; i++) {
//             data[i] = other.data[i];
//         }
//     }
// }

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
    // std::cout << "Vector::magnitude() called\n";
    double sum = 0.0;
    for (unsigned int i = 0; i < this-> rows; i++) {
        // std::cout << this->rows << "\n";
        sum += ((*this)(i,0) * (*this)(i,0));
        // std::cout << "summing\n";
        // std::cout << sum << "\n";
    }
    // std::cout << "Vector magnitude complete, returning\n";
    return (this->squareRoot(sum));

}

double Vector::squareRoot(unsigned int num) const {
    if (num < 0) {
        return -1.0;  // Error code
    }
    else if (num == 0) {
        return 0.0;
    }
    else if (num == 1) {
        return 1.0;
    }
    // std::cout << "Square root called\n";
    double guess = num / 2.0;
    double tolerance = 0.000001;  // Accuracy of the result
    // std::cout << "Initial guess: " << guess << "\n";

    while (true) {
        double betterGuess = (guess + num / guess) / 2.0;
        // std::cout << "guess: " << guess << "\n";
        if (abs(betterGuess - guess) < tolerance) {
            // std::cout << "Square root complete, returning\n";
            break;
        }
        guess = betterGuess;
    }

    return guess;
}

Vector Vector::normalize() const {
    double length = this->magnitude();
    if (length == 0) {
        return Vector(0, 0.0);
    }
    Matrix v = this->multiply(1.0/length);
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
