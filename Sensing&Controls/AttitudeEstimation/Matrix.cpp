#include <stdexcept>
#include "Matrix.h"
#include "Quaternion.h"
#include <cmath>

//Initialize data pointer to a double which allows for efficient array creation etc. and rows and cols
    
//Matrix is stored in a 1D array because of memory and elements are stored in the array by row
//Default constructor
Matrix::Matrix() : rows (0), cols (0), data(nullptr) {}

//Matrix constructor for all 0s, all 1s, or all constants
Matrix::Matrix(unsigned int rows, unsigned int cols, double initVal = 0)
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


Matrix::Matrix(const Matrix& other) : rows(other.getRows()), cols(other.getCols()) {
    if (other.data) {
        data = new double[rows * cols];
        for (unsigned int i = 0; i < rows * cols; ++ i) {
            data[i] = other.data[i];
        }
    }
}
        
Matrix::~Matrix() {
    delete[] data;
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

//defining matrix addition
Matrix Matrix::operator+(const Matrix& other) const { 
    if (rows != other.rows || cols != other.cols) return Matrix(0.0); //Error cannot add if not equal dimensions
    Matrix result(rows, cols, 0.0);
    for (unsigned int i = 0; i < rows * cols; ++i) {
        result.data[i] = data[i] + other.data[i];
    }
    return result;
}

//defining matrix subtraction
Matrix Matrix::operator-(const Matrix& other) const {  
    if (rows != other.rows || cols != other.cols) return Matrix(0.0); //Error cannot add if not equal dimensions
    Matrix result(rows, cols, 0.0);
    for (unsigned int i = 0; i < rows * cols; ++i) {
        result.data[i] = data[i] - other.data[i];
    }
    return result;
}

//defining matrix multiplication
Matrix Matrix::operator*(const Matrix& other) const {
    if (cols != other.rows) return Matrix(0,0,0); //Error can't multiply matrix of these dimensions

    Matrix result(rows, other.cols, 0.0);
    for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < other.cols; ++j) {
            double sum = 0.0;
            for (unsigned int k = 0; k < cols; ++k) {
                sum += (*this)(i,k) * other(k,j);
            }
            result(i, j) = sum;
        }
    }
    return result;
}

//defining matrix multiplication by scalar on the right
Matrix Matrix::operator*(double scalar) const {
    Matrix result(rows, cols, 0.0);
    for (unsigned int i = 0; i < rows * cols; ++i) {
        result.data[i] = data[i] * scalar;
    } 
    return result;
}

//defining matrix multiplication by scalar on the left
Matrix operator*(double scalar, const Matrix& m) {
    return m * scalar;
}


Matrix Matrix::transpose() const {
    Matrix result(cols, rows);
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
    
    Matrix result(rows); //create identity matrix
    Matrix temp = *this;
    
    
    // Compute X^k using exponentiation by squaring
    while (k > 0) {
        if (k % 2 == 1) {
            result = result * temp;
        }
        temp = temp * temp;
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
    Matrix term(rows, cols);   // Temporary matrix for intermediate results
        
    // Initialize result as the identity matrix
    for (unsigned int i = 0; i < rows; ++i) {
        result(i, i) = 1.0;
    }
        
    // Compute e^X using Taylor series expansion
    for (unsigned int k = 1; k <= terms; ++k) {
        term = this->power(k) * (1.0 / factorial(k));
        result = result + term;
    }
        
    return result;
}

bool Matrix::isEqualTo(const Matrix& other) const {
    if (rows != other.rows || cols != other.cols) {
        return false;
    }
    for (unsigned int i = 0; i < rows * cols; ++i) {
        if (data[i] != other.data[i]) {
            return false;
        }
    }
    return true;
}
      
Quaternion Matrix::toQuaternion() const {
    if (rows != 3 || cols != 3) {
        throw std::invalid_argument("Matrix must be 3x3 to convert to a Quaternion.");
    }

    double trace = (*this)(0, 0) + (*this)(1, 1) + (*this)(2, 2);
    double w, x, y, z;

    if (trace > 0) {
        double s = sqrt(trace + 1.0) * 2.0;  // 4w
        w = 0.25 * s;
        x = ((*this)(2, 1) - (*this)(1, 2)) / s;
        y = ((*this)(0, 2) - (*this)(2, 0)) / s;
        z = ((*this)(1, 0) - (*this)(0, 1)) / s;
    } else {
        if ((*this)(0, 0) > (*this)(1, 1) && (*this)(0, 0) > (*this)(2, 2)) {
            double s = sqrt(1.0 + (*this)(0, 0) - (*this)(1, 1) - (*this)(2, 2)) * 2.0;  // 4x
            w = ((*this)(2, 1) - (*this)(1, 2)) / s;
            x = 0.25 * s;
            y = ((*this)(0, 1) + (*this)(1, 0)) / s;
            z = ((*this)(0, 2) + (*this)(2, 0)) / s;
        } else if ((*this)(1, 1) > (*this)(2, 2)) {
            double s = sqrt(1.0 + (*this)(1, 1) - (*this)(0, 0) - (*this)(2, 2)) * 2.0;  // 4y
            w = ((*this)(0, 2) - (*this)(2, 0)) / s;
            x = ((*this)(0, 1) + (*this)(1, 0)) / s;
            y = 0.25 * s;
            z = ((*this)(1, 2) + (*this)(2, 1)) / s;
        } else {
            double s = sqrt(1.0 + (*this)(2, 2) - (*this)(0, 0) - (*this)(1, 1)) * 2.0;  // 4z
            w = ((*this)(1, 0) - (*this)(0, 1)) / s;
            x = ((*this)(0, 2) + (*this)(2, 0)) / s;
            y = ((*this)(1, 2) + (*this)(2, 1)) / s;
            z = 0.25 * s;
        }
    }

    return Quaternion(w, x, y, z);
}