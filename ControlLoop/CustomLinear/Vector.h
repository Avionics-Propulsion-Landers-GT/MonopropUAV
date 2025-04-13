#ifndef VECTOR_H
#define VECTOR_H
#include "Matrix.h"

// struct FromArrayTag {};
// constexpr FromArrayTag FromArray{};

class Vector : public Matrix {
    // protected:
    //     double* data;
    //     unsigned int rows, cols;

    public:
        Vector(unsigned int rows, double initVal)
            : Matrix (rows, 1, initVal) {}
        // deep copy constructor
        // Vector(const Vector &other) : rows(other.getRows()), data(nullptr) {};
        
        Vector(const Matrix& mat) : Matrix(mat) {
                
            if (mat.getCols() == 1 && mat.getRows() > 0) {
            } else {
                *this = Vector(0,0.0); 
            }
        }

        double& operator[](unsigned int row) {
            return (*this)(row, 0);
        }
        const double& operator[](unsigned int row) const {
            return(*this)(row, 0);
        }

        double dotProduct(const Vector&other) const;
        double magnitude() const;
        Vector normalize() const;
        Vector crossProduct(const Vector&other) const;
        double squareRoot(unsigned int num) const;
};
#endif