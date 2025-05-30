//
// Created by Sarang Suman on 3/6/25.
//

#include "Matrix.h"
#include "Quaternion.h"
#include "Vector.h"
#include <iostream>


const int rows = 3;
const int cols = 3;
const int initialValue = 1;

int main() {
    Matrix matrix = Matrix(rows, cols, initialValue);

    // Print the initial matrix
    std::cout << "Initial Matrix:" << std::endl;
    for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < cols; ++j) {
            std::cout << matrix(i, j) << " ";
        }
        std::cout << std::endl;
    }
    //identity matrix
    Matrix idMatrix = Matrix(rows);
    std::cout << "Identity Matrix:" << std::endl;
    for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < rows; ++j) {
            std::cout << idMatrix(i, j) << " ";
        }
        std::cout << std::endl;
    }
    //det if identity matrix
    double det2 = idMatrix.determinant();
    std::cout << "\nIdentity Determinant: " << det2 << std::endl;

    // Test addition
    Matrix matrix2 = Matrix(rows, cols, 2);
    std::cout << "Second Matrix:" << std::endl;
    for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < cols; ++j) {
            std::cout << matrix2(i, j) << " ";
        }
        std::cout << std::endl;
    }

    Matrix sumMatrix = matrix + matrix2;
    std::cout << "\nSum Matrix:" << std::endl;
    for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < cols; ++j) {
            std::cout << sumMatrix(i, j) << " ";
        }
        std::cout << std::endl;
    }

    // Test multiplication by scalar
    Matrix scalarMatrix = matrix*2.0;
    std::cout << "\nScalar Multiplication Matrix:" << std::endl;
    for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < cols; ++j) {
            std::cout << scalarMatrix(i, j) << " ";
        }
        std::cout << std::endl;
    }

    // Test transpose
    Matrix transposedMatrix = matrix.transpose();
    std::cout << "\nTransposed Matrix:" << std::endl;
    for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < cols; ++j) {
            std::cout << transposedMatrix(i, j) << " ";
        }
        std::cout << std::endl;
    }

    // Test determinant
    double det = matrix.determinant();
    std::cout << "\nDeterminant: " << det << std::endl;

    // Test inverse
    if (matrix.isInvertible()) {
        Matrix inverseMatrix = matrix.inverse();
        std::cout << "\nInverse Matrix:" << std::endl;
        for (unsigned int i = 0; i < rows; ++i) {
            for (unsigned int j = 0; j < cols; ++j) {
                std::cout << inverseMatrix(i, j) << " ";
            }
            std::cout << std::endl;
        }
    } else {
        std::cout << "\nMatrix is not invertible." << std::endl;
    }

    // Test Quaternion operations
    Quaternion q1(1.0, 0.0, 1.0, 0.0);
    Quaternion q2(1.0, 0.5, 0.5, 0.75);

    // Test Quaternion addition
    Quaternion qSum = q1+q2;
    std::cout << "\nQuaternion Sum: (" << qSum[0] << ", " << qSum[1] << ", " << qSum[2] << ", " << qSum[3] << ")" << std::endl;

    // Test Quaternion multiplication
    Quaternion qProduct = q1*q2;
    std::cout << "Quaternion Product: (" << qProduct[0] << ", " << qProduct[1] << ", " << qProduct[2] << ", " << qProduct[3] << ")" << std::endl;

    // Test Quaternion norm
    double qNorm = q1.magnitude();
    std::cout << "Quaternion Norm: " << qNorm << std::endl;

    // Test Quaternion to rotation matrix
    Matrix rotationMatrix = q1.toRotationMatrix();
    std::cout << "Quaternion to Rotation Matrix:" << std::endl;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            std::cout << rotationMatrix(i,j) << " ";
        }
        std::cout << std::endl;
    }

    // Test Vector operations
    Vector v1(1, 3.0);
    Vector v2(2, 6.9);

    // Test Vector dot product
    double vDot = v1.dotProduct(v2);
    std::cout << "Vector Dot Product: " << vDot << std::endl;


    return 0;
};