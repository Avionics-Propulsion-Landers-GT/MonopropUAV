//
// Created by Sarang Suman on 3/6/25.
//

#include "Matrix.h"
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

        // Test addition
        Matrix matrix2 = Matrix(rows, cols, 2);
        Matrix sumMatrix = matrix.add(matrix2);
        std::cout << "\nSum Matrix:" << std::endl;
        for (unsigned int i = 0; i < rows; ++i) {
            for (unsigned int j = 0; j < cols; ++j) {
                std::cout << sumMatrix(i, j) << " ";
            }
            std::cout << std::endl;
        }

        // Test multiplication by scalar
        Matrix scalarMatrix = matrix.multiply(2.0);
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

        return 0;

}