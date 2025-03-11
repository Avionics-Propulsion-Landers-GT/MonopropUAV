#ifndef VECTOR_H
#define VECTOR_H
#include "Matrix.h";
class Vector : public Matrix {
    public:
        Vector(unsigned int rows, double initVal = 0.0) 
            : Matrix (rows, 1, initVal) {}
        
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
};
#endif