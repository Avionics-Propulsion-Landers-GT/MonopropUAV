#ifndef MATRIX_H
#define MATRIX_H

#include <stdexcept>
#include <cmath>
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
        Matrix block(int i, int j, int rows, int cols) const;
        static Matrix fromArray(int rows, int cols, const double* data, bool colMajor);
        void toArray(double* out, bool colMajor) const;

        // 
    

};

Matrix formHouseholderVector(Matrix& R, unsigned int k, unsigned int m, unsigned int n);
Matrix applyHouseholderToQ(const Matrix& Q_in, const Matrix& v, double beta, unsigned int k);
Matrix computeEigenvaluesShiftedQR(const Matrix& A_in, int maxIter, double tol);
Matrix reshapeVectorToMatrix(const Matrix& v, unsigned int n);
void applyGivensRotation(Matrix& T, Matrix& Q, int i);
Matrix reActivate(const Matrix& activations);
void QRDecomposition(const Matrix& A, Matrix& Q, Matrix& R);
Matrix Identity(unsigned int n);
void swapBlocks(Matrix& Q, Matrix& T, int i);
void ThinQR(const Matrix& A, Matrix& Q, Matrix& R);
double colVectorNorm(const Matrix& v);
void schurDecomposition(const Matrix& A_in, Matrix& Q_out, Matrix& T_out, int maxIter, double tol);
double wilkinsonShift2x2(double a, double b, double c, double d);
Matrix buildHamiltonian(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R);
bool isStableBlock(const Matrix& T, int i);
void reorderSchurForm(Matrix& T, Matrix& Q);
void shiftBlockDown(Matrix& T, Matrix& Q, int i);
Matrix solveSylvester(const Matrix& A11, const Matrix& A22, const Matrix& A12);
void applyGivensRotation(Matrix& T, Matrix& Q, int i, int j, double c, double s);
void computeBlockReorderQR(const Matrix& T, int i, int p, int q, Matrix& Q_out );
Matrix completeToOrthonormal(const Matrix& Qthin);

#endif