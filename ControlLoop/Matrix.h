#ifndef MATRIX_H
#define MATRIX_H

#include <stdexcept>
class Matrix {
    protected:
        double* data;
        unsigned int rows, cols;

    public:
        Matrix(unsigned int rows, unsigned int cols, double initVal);
        Matrix(unsigned int n);
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
        double determinant() const;
        double cofactor (unsigned int row, unsigned int col) const;
        Matrix getSubMatrix(unsigned int row, unsigned int col) const;
        void print() const;
        bool luDecompose(Matrix& L, Matrix& U, Matrix& P) const;
        Matrix luInverse() const;
        Matrix pseudoInverseJacobi(double rankEps, int maxIter) const;
        Matrix pseudoInverseAuto(double rankEps, int maxIter) const;
        void thinJacobiSVD(Matrix& U, Matrix& Sigma, Matrix& V, double rankEps, int maxIter) const;
        Matrix controllabilityMatrix(const Matrix& B) const;
        double frobeniusNorm() const;
        bool isControllable(const Matrix& B, double tol) const;
        unsigned int rank(double tol) const;
        Matrix pseudoInverse() const;
        void sanitizeNaNs();
        bool isInvertible() const;
        Matrix power(unsigned int k) const;
        double factorial(unsigned int k) const;
        Matrix exp(unsigned int terms) const;

};

Matrix reshapeVectorToMatrix(const Matrix& v, unsigned int n);
Matrix solveCARE_FixedPoint(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R, const double & alpha);
Matrix solveCARE_diagonal(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R);
Matrix reActivate(const Matrix& activations);

#endif