#include <stdexcept>
#include <iostream>
#include "Matrix.h"
#include "Vector.h"

//Initialize data pointer to a double which allows for efficient array creation etc. and rows and cols
    
//Matrix is stored in a 1D array because of memory and elements are stored in the array by row
//Matrix constructor for all 0s, all 1s, or all constants
Matrix::Matrix(unsigned int rows, unsigned int cols, double initVal)
    : rows(rows), cols(cols), data(nullptr) {
        if (rows > 0 && cols > 0) {
            data = new double[rows * cols];
            for (unsigned int i = 0; i < rows * cols; ++i) {
                data[i] = initVal;
            }
        }
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

Matrix::Matrix(const Matrix& other) : rows(other.getRows()), cols(other.getCols()) {
    if (other.data) {
        data = new double[rows * cols];
        for (unsigned int i = 0; i < rows * cols; ++ i) {
            data[i] = other.data[i];
        }
    }
}


// Spencer -- Added copy assignment operator
Matrix& Matrix::operator=(const Matrix& other) {
    if (this == &other) return *this;  // Protect against self-assignment

    // Clean up existing memory
    delete[] data;

    // Copy dimensions and allocate new memory
    rows = other.rows;
    cols = other.cols;
    data = new double[rows * cols];

    // Deep copy the contents
    for (unsigned int i = 0; i < rows * cols; ++i) {
        data[i] = other.data[i];
    }

    return *this;
}
       
// Spencer -- Protected destructor, I was getting errors
Matrix::~Matrix() {
    if (data != nullptr) {
        delete[] data;
        data = nullptr;
    }
}
        
//Reference to a matrix entry at row by col (allows for changing that entry)
double& Matrix::operator()(unsigned int row, unsigned int col) {
    static double dummyVar = 0.0;
        if (row >= rows || col >= cols) {
            return dummyVar;
        }
        return data[row * cols + col];
}
//reference to a matrix entry at row by col (allows for value to be accessed but not to be changed as const promises to not change the object's state)
const double& Matrix::operator()(unsigned int row, unsigned int col) const {
    return data[row * cols + col];
}

Matrix Matrix::add(const Matrix& other) const {
    if (rows != other.rows || cols != other.cols) return Matrix(0.0); //Error cannot add if not equal dimensions

    Matrix result(rows, cols, 0);
    for (unsigned int i = 0; i < rows * cols; ++i)
        result.data[i] = data[i] + other.data[i];
    return result;
}

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

Matrix Matrix::multiply(double scalar) const {
    Matrix result(rows, cols, 0.0);
    for (unsigned int i = 0; i < rows * cols; ++i) {
        result.data[i] = data[i] * scalar;
    }
    return result;
}

Matrix Matrix::transpose() const {
    Matrix result(cols, rows, 0);
    for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < cols; ++j) {
            result(j, i) = (*this)(i,j);
        }
    }
    return result;
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
    Matrix subMatrix (rows - 1, cols - 1, 0);
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

Matrix Matrix::inverse() const {
    if (rows != cols) {
        return Matrix(0,0, 0); //can't compute inverse if matrix is not square
    } else {
        double det = determinant();
        if (det == 0) {
            return Matrix(0,0, 0);
        }

        Matrix adj(rows, cols, 0);
        for (unsigned int i = 0; i < rows; ++i) {
            for (unsigned int j = 0; j < rows; ++j) {
                adj(j, i) = cofactor(i,j);
            }
        }
        Matrix inv(rows, cols, 0);
        for (unsigned int i = 0; i < rows; ++i) {
            for (unsigned int j = 0; j < cols; ++j) {
                inv(i,j) = adj(i,j) / det;
            }
        }
        return inv;
    }
}

// Spencer ~ Created fast inverse with LU Dc
Matrix Matrix::luInverse() const {
    if (rows != cols) {
        std::cout << "[ERROR] Matrix must be square to invert.\n";
        return Matrix(0, 0, 0);
    }

    unsigned int n = rows;
    Matrix L(n, n, 0.0);
    Matrix U(n, n, 0.0);
    Matrix P(n, n, 0.0);

    if (!luDecompose(L, U, P)) {
        std::cerr << "[ERROR] LU decomposition failed. Matrix may be singular.\n";
        return Matrix(0, 0, 0);
    }

    Matrix inverse(n, n, 0.0);
    Matrix I(n); // identity matrix

    for (unsigned int col = 0; col < n; ++col) {
        // Extract column vector of identity matrix
        Vector e(n, 0.0);
        e(col, 0) = 1.0;

        // Apply permutation: Pb = P * e
        Vector Pb = Vector(P.multiply(e));

        // Solve Ly = Pb via forward substitution
        Vector y(n, 0.0);
        for (unsigned int i = 0; i < n; ++i) {
            double sum = 0.0;
            for (unsigned int j = 0; j < i; ++j)
                sum += L(i, j) * y(j, 0);
            y(i, 0) = (Pb(i, 0) - sum) / L(i, i);
        }

        // Solve Ux = y via back substitution
        Vector x(n, 0.0);
        for (int i = n - 1; i >= 0; --i) {
            double sum = 0.0;
            for (unsigned int j = i + 1; j < n; ++j)
                sum += U(i, j) * x(j, 0);
            x(i, 0) = (y(i, 0) - sum) / U(i, i);
        }

        // Copy x as column into inverse
        for (unsigned int row = 0; row < n; ++row) {
            inverse(row, col) = x(row, 0);
        }
    }

    return inverse;
}


bool Matrix::luDecompose(Matrix& L, Matrix& U, Matrix& P) const {
    if (rows != cols) return false;

    unsigned int n = rows;
    P = Matrix(n); // Identity matrix
    Matrix A(*this);
    L = Matrix(n, n, 0.0);
    U = Matrix(n, n, 0.0);

    // Partial pivoting
    for (unsigned int i = 0; i < n; ++i) {
        // Find pivot
        double max_val = std::abs(A(i, i));
        unsigned int pivot = i;
        for (unsigned int j = i + 1; j < n; ++j) {
            if (std::abs(A(j, i)) > max_val) {
                max_val = std::abs(A(j, i));
                pivot = j;
            }
        }

        if (max_val == 0.0) return false; // Singular matrix

        // Swap rows in A and P
        if (pivot != i) {
            for (unsigned int k = 0; k < n; ++k) {
                std::swap(A(i, k), A(pivot, k));
                std::swap(P(i, k), P(pivot, k));
            }
        }

        // Decompose
        for (unsigned int j = i; j < n; ++j) {
            double sum = 0.0;
            for (unsigned int k = 0; k < i; ++k)
                sum += L(i, k) * U(k, j);
            U(i, j) = A(i, j) - sum;
        }

        for (unsigned int j = i; j < n; ++j) {
            if (i == j)
                L(i, i) = 1.0;
            else {
                double sum = 0.0;
                for (unsigned int k = 0; k < i; ++k)
                    sum += L(j, k) * U(k, i);
                L(j, i) = (A(j, i) - sum) / U(i, i);
            }
        }
    }

    return true;
}


bool Matrix::isInvertible() const {
    if (rows != cols) {
        return false;
    }
    return determinant() != 0.0;
}

Matrix Matrix::power(unsigned int k) const {
    if (rows != cols) {
        throw std::invalid_argument("Matrix must be square for exponentiation");
    }
    
    Matrix result(rows);
    Matrix temp = *this;
    
    
    // Compute X^k using exponentiation by squaring
    while (k > 0) {
        if (k % 2 == 1) {
            result = result.multiply(temp);
        }
        temp = temp.multiply(temp);
        k /= 2;
    }
    return result;
}

double Matrix::factorial(unsigned int k) const {
    if (k == 0 || k == 1) {
        return 1.0;
    }
    return k * factorial(k - 1);
}

Matrix Matrix::exp(unsigned int terms) const {
    if (rows != cols) {
        throw std::invalid_argument("Matrix must be square for exponentiation");
    }
        
    Matrix result(rows); // Initialize result as the identity matrix
    Matrix term(rows, cols, 0);   // Temporary matrix for intermediate results
        
    // Initialize result as the identity matrix
    for (unsigned int i = 0; i < rows; ++i) {
        result(i, i) = 1.0;
    }
        
    // Compute e^X using Taylor series expansion
    for (unsigned int k = 1; k <= terms; ++k) {
        term = this->power(k).multiply(1.0 / factorial(k));
        result = result.add(term);
    }
        
    return result;
}
