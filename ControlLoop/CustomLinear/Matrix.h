#ifndef MATRIX_H
#define MATRIX_H

#include <stdexcept>
#include <cmath>

// Dummy struct to tell constructor it's reading from array
struct FromArrayTag {};
constexpr FromArrayTag FromArray{};

class Matrix {
    protected:
        double* data;
        unsigned int rows, cols;

    public:
        Matrix();
        Matrix(unsigned int rows, unsigned int cols, double initVal);
        Matrix(unsigned int n);
        Matrix(unsigned int rows, unsigned int cols, const double* data, FromArrayTag);
        Matrix(const Matrix& other);
        ~Matrix();
        Matrix& operator=(const Matrix& other);


        double& operator()(unsigned int row, unsigned int col);
        const double& operator()(unsigned int row, unsigned int col) const;

        unsigned int getRows() const {
            return this->rows;
        }
        unsigned int getCols() const {
            return this->cols;
        }


        Matrix add(const Matrix& other) const;
        Matrix subtract(const Matrix& other) const;
        Matrix multiply(const Matrix& other) const;
        Matrix multiply(double scalar) const;
        Matrix transpose() const;
        Matrix inverse() const;
        double determinant() const;
        double cofactor (unsigned int row, unsigned int col) const;
        Matrix getSubMatrix(unsigned int row, unsigned int col) const;
        void print() const;
        Matrix pseudoInverseJacobi(double rankEps, int maxIter) const;
        void thinJacobiSVD(Matrix& U, Matrix& Sigma, Matrix& V, double rankEps, int maxIter) const;
        Matrix controllabilityMatrix(const Matrix& B) const;
        unsigned int rank(double tol) const;
        void sanitizeNaNs();
        static Matrix fromArray(int rows, int cols, const double* data, bool colMajor);
        void toArray(double* out, bool colMajor) const;

};

Matrix reActivate(const Matrix& activations);

#endif