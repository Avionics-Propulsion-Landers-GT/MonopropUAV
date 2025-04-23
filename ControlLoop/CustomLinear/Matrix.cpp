#include <stdexcept>
#include <iostream>
#include <iomanip> 
#include "Matrix.h"
#include "Vector.h"
#include <cmath>
#include <limits>
#include <cassert>


//Initialize data pointer to a double which allows for efficient array creation etc. and rows and cols
    
//Matrix is stored in a 1D array because of memory and elements are stored in the array by row
// default constructor
Matrix::Matrix() : rows (0), cols (0), data(nullptr) {}

Matrix::Matrix(unsigned int rows, unsigned int cols, double val = 0)
    : rows(rows), cols(cols) {
    size_t total = rows * cols;
    // std::cout << "[Matrix] ctor alloc: " << total << " elements\n";
    data = (total > 0) ? new double[total] : nullptr;
    for (size_t i = 0; i < total; ++i) data[i] = val;
}

//Identity Matrix Constructor
Matrix::Matrix(unsigned int n)
    : rows(n), cols(n), data(nullptr) {
        //may need exception here for 0 case
        if (n > 0) {
            data = new double[n * n]();

            for (unsigned int i = 0; i < n; ++i) {
                data[i * n + i] = 1.0;
            }
        }
}

Matrix::Matrix(unsigned int r, unsigned int c, const double* d, FromArrayTag)
    : rows(r), cols(c), data(nullptr) {
    if (r == 0 || c == 0) {
        throw std::invalid_argument("Rows and columns must be greater than zero");
    }
    if (d == nullptr) {
        throw std::invalid_argument("Data pointer cannot be null");
    }
    data = new double[r * c];
    for (unsigned int i = 0; i < r * c; ++i) {
        data[i] = d[i];
    }
}

// GetRows, GetCols
Matrix::Matrix(const Matrix& other) : rows(other.getRows()), cols(other.getCols()) {
    if (other.data) {
        data = new double[rows * cols];
        for (unsigned int i = 0; i < rows * cols; ++ i) {
            data[i] = other.data[i];
        }
    }
}

// Copy Ass. Operator
Matrix& Matrix::operator=(const Matrix& other) {
    if (this == &other) return *this;

    if (data != nullptr) {
        // std::cout << "[Matrix] Freeing data at: " << static_cast<void*>(data)
        //         << " with size: " << rows << " x " << cols << "\n";
        delete[] data;
    }

    rows = other.rows;
    cols = other.cols;

    if (rows == 0 || cols == 0) {
        data = nullptr;
        return *this;
    }

    // std::cout << "[Matrix] Allocating data: " << rows << " x " << cols << "\n";
    data = new double[rows * cols];

    for (unsigned int i = 0; i < rows * cols; ++i) {
        data[i] = other.data[i];
    }

    return *this;
}
    
// Destructor
Matrix::~Matrix() {
    delete[] data;  // delete[] is safe even on nullptr
    data = nullptr;
}
    
// Assignment Operator
double& Matrix::operator()(unsigned int row, unsigned int col) {
    if (row >= rows || col >= cols) {
        throw std::out_of_range(
            "[Matrix::operator()] Write access out of bounds at (" + 
            std::to_string(row) + ", " + std::to_string(col) + 
            ") for matrix of size " + std::to_string(rows) + "x" + std::to_string(cols)
        );
    }
    return data[row * cols + col];
}

// Read operator
const double& Matrix::operator()(unsigned int row, unsigned int col) const {
    if (row >= rows || col >= cols) {
        throw std::out_of_range(
            "[Matrix::operator() const] Read access out of bounds at (" + 
            std::to_string(row) + ", " + std::to_string(col) + 
            ") for matrix of size " + std::to_string(rows) + "x" + std::to_string(cols)
        );
    }
    return data[row * cols + col];
}

// Add
Matrix Matrix::add(const Matrix& other) const {
    if (rows != other.rows || cols != other.cols) return Matrix(0.0); //Error cannot add if not equal dimensions

    Matrix result(rows, cols, 0);
    for (unsigned int i = 0; i < rows * cols; ++i)
        result.data[i] = data[i] + other.data[i];
    return result;
}

// Subtract
Matrix Matrix::subtract(const Matrix& other) const {
    if (rows != other.rows || cols != other.cols) return Matrix(0.0); //Error cannot subtract if not equal dimensions

    Matrix C(rows, cols, 0.0);
    for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < cols; ++j) {
            C(i, j) = operator()(i, j) - other(i, j);
        }
    }
    return C;
}

// Multiply
Matrix Matrix::multiply(const Matrix& other) const {
    if (cols != other.rows) return Matrix(0.0); //Error can't multiply matrix of these dimensions

    Matrix result(rows, other.cols, 0.0);
    for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < other.cols; ++j) {
            double sum = 0.0;
            for (unsigned int k = 0; k < cols; ++k) {
                sum += operator()(i,k) * other(k,j);
            }
            result(i, j) = sum;
        }
    }
    return result;
}

// Scalar multiply (overloaded)
Matrix Matrix::multiply(double scalar) const {
    Matrix result(rows, cols, 0.0);
    for (unsigned int i = 0; i < rows * cols; ++i) {
        result.data[i] = data[i] * scalar;
    }
    return result;
}

double frobeniusNorm(Matrix a) {
    double sum = 0.0;
    unsigned int rows = a.getRows();
    unsigned int cols = a.getCols();

    for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < cols; ++j) {
            double val = a(i, j);
            sum += val * val;
        }
    }
    return std::sqrt(sum);
}


// Transpose
Matrix Matrix::transpose() const {
    Matrix result(cols, rows, 0);
    for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < cols; ++j) {
            result(j, i) = (*this)(i,j);
        }
    }
    return result;
}  

Matrix Matrix::inverse() const {
    if (rows != cols) {
        return Matrix(0,0,0); //can't compute inverse if matrix is not square
    } else {
        double det = determinant();
        if (det == 0) {
            return Matrix(0,0,0);
        }

        Matrix adj(rows, cols);
        for (unsigned int i = 0; i < rows; ++i) {
            for (unsigned int j = 0; j < rows; ++j) {
                adj(j, i) = cofactor(i,j);
            }
        }
        Matrix inv(rows, cols);
        for (unsigned int i = 0; i < rows; ++i) {
            for (unsigned int j = 0; j < cols; ++j) {
                inv(i,j) = adj(i,j) / det;
            }
        }
        return inv;
    }
}

double Matrix::determinant() const {
    if (rows != cols) {
        return 0;
    }
    //2x2: det = ad - bc
    if (rows == 2) {
        return (*this)(0,0) * (*this)(1,1) - (*this)(0,1) * (*this)(1,0);
    }

    double det = 0;
    for (unsigned int i = 0; i < cols; ++i) {
        det += (*this)(0,i) * cofactor (0,i);
    }
    return det;
}

double Matrix::cofactor (unsigned int row, unsigned int col) const {
    Matrix subMatrix = getSubMatrix(row, col);
    return ((row + col) % 2 == 0 ? 1.0 : -1.0) * subMatrix.determinant();
}

Matrix Matrix::getSubMatrix(unsigned int row, unsigned int col) const {
    Matrix subMatrix (rows - 1, cols - 1);
    unsigned int subRow = 0;
    for (unsigned int i = 0; i < rows; ++i) {
        if (i == row) {
            continue;
        }
        unsigned int subCol = 0;
        for (unsigned int j = 0; j < cols; ++j) {
            if (j == col) {
                continue;
            }
            subMatrix(subRow, subCol) = (*this)(i,j);
            subCol++;
        }
        subRow++;
    }
    return subMatrix;
}

void Matrix::swapRows(unsigned int r1, unsigned int r2) {
    if (r1 >= rows || r2 >= rows) {
        throw std::out_of_range("Row index out of bounds in swapRows");
    }

    for (unsigned int col = 0; col < cols; ++col) {
        double temp = (*this)(r1, col);
        (*this)(r1, col) = (*this)(r2, col);
        (*this)(r2, col) = temp;
    }
}

void Matrix::print() const {
    unsigned int precision = 4;
    for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < cols; ++j) {
            std::cout << std::setw(10) << std::fixed << std::setprecision(precision) << (*this)(i, j) << " ";
        }
        std::cout << std::endl;
    }
}

Matrix Matrix::fromArray(int rows, int cols, const double* data, bool colMajor) {
    Matrix result(rows, cols, 0.0);
    if (colMajor) {
        for (int j = 0; j < cols; ++j)
            for (int i = 0; i < rows; ++i)
                result(i, j) = data[j * rows + i];
    } else {
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)
                result(i, j) = data[i * cols + j];
    }
    return result;
}

void Matrix::toArray(double* out, bool colMajor) const {
    if (colMajor) {
        for (int j = 0; j < this->cols; ++j)
            for (int i = 0; i < this->rows; ++i)
                out[j * this->rows + i] = (*this)(i, j);
    } else {
        for (int i = 0; i < this->rows; ++i)
            for (int j = 0; j < this->cols; ++j)
                out[i * this->cols + j] = (*this)(i, j);
    }
}

Matrix Matrix::pseudoInverseJacobi(double rankEps, int maxIter) const
{
    unsigned int m = this->rows; // e.g. 12
    unsigned int n = this->cols; // e.g. 7
    bool flipped = false;
    if (m < n) {
        Matrix At = *this;
        Matrix At_orig = *this;
        Matrix A = At.transpose();
        Matrix A_orig = A_orig.transpose();
        flipped = true;
    } else {
        Matrix A = *this;
        Matrix A_orig = *this;
    }

    Matrix A = *this;
    Matrix A_orig = *this;
    Matrix V(n, n, 0.0);

    for (unsigned int i = 0; i < n; ++i) {
        V(i, i) = 1.0;
    }

    for (int iter = 0; iter < maxIter; ++iter)
    {
        bool changed = false;
        for (unsigned int col = 0; col < n; ++col) {
            double normCol = 0.0;
            for (unsigned int row = 0; row < m; ++row) {
                double val = A(row, col);
                normCol += val*val;
            }
            
        }
        for (unsigned int p = 0; p < n - 1; ++p) {
            for (unsigned int q = p + 1; q < n; ++q) {
                double app = 0.0, aqq = 0.0, apq = 0.0;
                for (unsigned int i = 0; i < m; ++i) {
                    double aip = A(i, p);
                    double aiq = A(i, q);
                    app += aip * aip;
                    aqq += aiq * aiq;
                    apq += aip * aiq;
                }
                double eps_orth = 1e-12 * std::sqrt(app * aqq);
                if (std::fabs(apq) <= eps_orth || app < 1e-30 || aqq < 1e-30)
                    continue;

                changed = true;
                double tau = (aqq - app) / (2.0 * apq);
                double t   = ( (tau >= 0.0) ? 1.0 : -1.0 )
                             / ( std::fabs(tau) + std::sqrt(1.0 + tau*tau) );
                double c = 1.0 / std::sqrt(1.0 + t*t);
                double s = c * t;
                for (unsigned int i = 0; i < m; ++i) {
                    double aip = A(i, p);
                    double aiq = A(i, q);
                    A(i, p) = c*aip - s*aiq;
                    A(i, q) = s*aip + c*aiq;
                }
                for (unsigned int i = 0; i < n; ++i) {
                    double vip = V(i, p);
                    double viq = V(i, q);
                    V(i, p) = c*vip - s*viq;
                    V(i, q) = s*vip + c*viq;
                }
            }
        }
        if (!changed) {
            break;
        }
    }
    Matrix sigma(n, 1, 0.0);
    for (unsigned int j = 0; j < n; ++j) {
        double sum = 0.0;
        for (unsigned int i = 0; i < m; ++i) {
            double val = A(i, j);
            sum += val*val;
        }
        sigma(j, 0) = std::sqrt(sum);
    }
    Matrix U(m, n, 0.0);
    for (unsigned int j = 0; j < n; ++j) {
        double sVal = sigma(j,0);
        if (sVal > rankEps) {
            // U(:,j) = A(:,j) / sVal
            for (unsigned int i = 0; i < m; ++i) {
                U(i, j) = A(i, j) / sVal;
            }
        } else {
            for (unsigned int i = 0; i < m; ++i) {
                U(i, j) = 0.0;
            }
        }
    }
    Matrix Sigma(n, n, 0.0);
    for (unsigned int j = 0; j < n; ++j) {
        double sVal = sigma(j,0);
        Sigma(j, j) = (sVal > rankEps) ? sVal : 0.0;
    }
    Matrix SigmaPlus(n, n, 0.0);
    for (unsigned int j = 0; j < n; ++j) {
        double sVal = Sigma(j, j);
        if (sVal > rankEps) {
            SigmaPlus(j, j) = 1.0 / sVal;
        }
    }

    Matrix Aplus = V.multiply(SigmaPlus).multiply(U.transpose()); // (n x m) = (7 x 12)

    if (flipped) {
        return Aplus.transpose();
    } else {
        return Aplus;
    }
}

Matrix Matrix::controllabilityMatrix(const Matrix& B) const {
    if (this->getRows() != this->getCols()) {
        throw std::runtime_error("A must be square for controllability matrix.");
    }
    if (this->getRows() != B.getRows()) {
        throw std::runtime_error("Matrix dimensions must agree (A.rows == B.rows).");
    }
    unsigned int n = this->getRows();
    unsigned int m = B.getCols();
    Matrix C(n, n * m, 0.0); // Controllability matrix: n x (n*m)
    Matrix A_pow = Matrix(n); // Start with A^0 = I
    for (unsigned int i = 0; i < n; ++i) {
        Matrix term = A_pow.multiply(B); // A^i * B
        if (term.getRows() != n || term.getCols() != m) {
            throw std::runtime_error("Unexpected shape in controllability computation.");
        }

        for (unsigned int r = 0; r < n; ++r) {
            for (unsigned int c = 0; c < m; ++c) {
                C(r, i * m + c) = term(r, c);
            }
        }
        A_pow = A_pow.multiply(*this); // Correct order for A^(i+1)
    }
    return C;
}

unsigned int Matrix::rank(double tol) const {
    unsigned int m = this->getRows();
    unsigned int n = this->getCols();
    unsigned int minDim = std::min(m, n);

    // Do thin SVD
    Matrix A = *this;
    Matrix U(m, n, 0.0);
    Matrix Sigma(n, 1, 0.0);
    Matrix V(n, n, 0.0);
    A.thinJacobiSVD(U, Sigma, V, tol, 100); 

    // Count singular values above threshold
    unsigned int rank = 0;
    for (unsigned int i = 0; i < minDim; ++i) {
        if (Sigma(i, i) > tol) {
            ++rank;
        }
    }

    return rank;
}

void Matrix::thinJacobiSVD(Matrix& U, Matrix& Sigma, Matrix& V, double rankEps, int maxIter) const
{
    unsigned int m = this->rows;
    unsigned int n = this->cols;
    bool flipped = false;
    if (m < n) {
        // std::cout << "[SVD] Flipping... \n";
        flipped = true;
    }
    Matrix A = *this;
    V = Matrix(n, n, 0.0);
    for (unsigned int i = 0; i < n; ++i)
        V(i, i) = 1.0;

    for (int iter = 0; iter < maxIter; ++iter) {
        bool changed = false;

        for (unsigned int p = 0; p < n - 1; ++p) {
            for (unsigned int q = p + 1; q < n; ++q) {
                double app = 0.0, aqq = 0.0, apq = 0.0;
                for (unsigned int i = 0; i < m; ++i) {
                    double aip = A(i, p), aiq = A(i, q);
                    app += aip * aip;
                    aqq += aiq * aiq;
                    apq += aip * aiq;
                }

                double eps_orth = 1e-12 * std::sqrt(app * aqq);
                if (std::fabs(apq) <= eps_orth || app < 1e-30 || aqq < 1e-30)
                    continue;

                changed = true;

                double tau = (aqq - app) / (2.0 * apq);
                double t = (tau >= 0.0 ? 1.0 : -1.0) / (std::fabs(tau) + std::sqrt(1.0 + tau * tau));
                double c = 1.0 / std::sqrt(1.0 + t * t);
                double s = c * t;

                for (unsigned int i = 0; i < m; ++i) {
                    double aip = A(i, p), aiq = A(i, q);
                    A(i, p) = c * aip - s * aiq;
                    A(i, q) = s * aip + c * aiq;
                }

                for (unsigned int i = 0; i < n; ++i) {
                    double vip = V(i, p), viq = V(i, q);
                    V(i, p) = c * vip - s * viq;
                    V(i, q) = s * vip + c * viq;
                }
            }
        }
        // Restore matrix to original dimensions
        if (flipped) this->transpose();

        if (!changed) break;
    }
    // Compute singular values
    Matrix sigma(n, 1, 0.0);
    for (unsigned int j = 0; j < n; ++j) {
        double sum = 0.0;
        for (unsigned int i = 0; i < m; ++i)
            sum += A(i, j) * A(i, j);
        sigma(j, 0) = std::sqrt(sum);
    }
    // Build U
    U = Matrix(m, n, 0.0);
    for (unsigned int j = 0; j < n; ++j) {
        double sVal = sigma(j, 0);
        if (sVal > rankEps) {
            for (unsigned int i = 0; i < m; ++i)
                U(i, j) = A(i, j) / sVal;
        }
    }
    // Build Sigma
    Sigma = Matrix(n, n, 0.0);
    for (unsigned int j = 0; j < n; ++j) {
        double sVal = sigma(j, 0);
        Sigma(j, j) = (sVal > rankEps) ? sVal : 0.0;
    }
    if (flipped) {
        Matrix temp = V;
        V = U.transpose();
        U = temp.transpose();
    }
}

Matrix reActivate(const Matrix& activations) {
    unsigned int m = activations.getRows();
    unsigned int n = activations.getCols();

    if (n != 1) {
        std::cerr << "[expandActivationsTall] Error: Expected column vector.\n";
        return Matrix(0, 0, 0); // or throw exception
    }

    // Count how many activations are non-zero (== 1.0)
    unsigned int activeCount = 0;
    for (unsigned int i = 0; i < m; ++i) {
        if (activations(i, 0) == 1.0) {
            ++activeCount;
        }
    }

    // Create matrix with shape (m x activeCount)
    Matrix result(m, activeCount, 0.0); // all zeros

    unsigned int col = 0;
    for (unsigned int i = 0; i < m; ++i) {
        if (activations(i, 0) == 1.0) {
            result(i, col) = 1.0;
            ++col;
        }
    }

    return result;
}

double Matrix::magnitude() const {
    double sum = 0.0;
    for (unsigned int i = 0; i < rows * cols; ++i) {
        sum += data[i] * data[i];
    }
    return std::sqrt(sum);
}

void Matrix::sanitizeNaNs() {
    double replacement = 0.0;
    for (unsigned int i = 0; i < this->rows; ++i) {
        for (unsigned int j = 0; j < this->cols; ++j) {
            if (std::isnan((*this)(i, j))) {
                (*this)(i, j) = replacement;
            }
        }
    }
}
