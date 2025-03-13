#ifndef MATRIX_H
#define MATRIX_H

#include <stdexcept>
class Matrix {
    protected:
        double* data;
        unsigned int rows, cols;

    public:
        Matrix();
        Matrix(unsigned int rows, unsigned int cols, double initVal);
        Matrix(unsigned int n);
        Matrix(unsigned int r, unsigned int c);
        ~Matrix();


        double& operator()(unsigned int row, unsigned int col);
        const double& operator()(unsigned int row, unsigned int col) const;

        unsigned int getRows() const {
            return this->rows;
        }
        unsigned int getCols() const {
            return this->cols;
        }

        //Matrix add(const Matrix& other) const;
        //Matrix multiply(const Matrix& other) const;
        //Matrix multiply(double scalar) const;
        Matrix operator+(const Matrix& other) const;
        Matrix operator-(const Matrix& other) const;
        Matrix operator*(const Matrix& other) const;
        Matrix operator*(double scalar) const;
        friend Matrix operator*(double scalar, const Matrix& m);

        Matrix transpose() const;
        double determinant() const;
        double cofactor (unsigned int row, unsigned int col) const;
        Matrix getSubMatrix(unsigned int row, unsigned int col) const;
        Matrix inverse() const;

        bool isInvertible() const;
        Matrix power(unsigned int k) const;
        double factorial(unsigned int k) const;
        Matrix exp(unsigned int terms) const;

};

#endif