#include <stdexcept>
#include <iostream>
#include <iomanip> 
#include "Matrix.h"
#include "Vector.h"
#include <cmath>

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

// Matrix Matrix::inverse() const {
//     if (rows != cols) {
//         return Matrix(0,0, 0); //can't compute inverse if matrix is not square
//     } else {
//         double det = determinant();
//         if (det == 0) {
//             return Matrix(0,0, 0);
//         }

//         Matrix adj(rows, cols, 0);
//         for (unsigned int i = 0; i < rows; ++i) {
//             for (unsigned int j = 0; j < rows; ++j) {
//                 adj(j, i) = cofactor(i,j);
//             }
//         }
//         Matrix inv(rows, cols, 0);
//         for (unsigned int i = 0; i < rows; ++i) {
//             for (unsigned int j = 0; j < cols; ++j) {
//                 inv(i,j) = adj(i,j) / det;
//             }
//         }
//         return inv;
//     }
// }

void Matrix::print() const {
    unsigned int precision = 4;
    for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < cols; ++j) {
            std::cout << std::setw(10) << std::fixed << std::setprecision(precision) << (*this)(i, j) << " ";
        }
        std::cout << std::endl;
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
        return this->pseudoInverse();
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

Matrix Matrix::pseudoInverseGolubKahan(double rankEps, int maxIter) const
{
    // ----------------------------------------------------
    // 0) Basic shape from *this
    unsigned int m = this->rows;
    unsigned int n = this->cols;

    // We'll store the SVD as: this = U (m x m) * diag(...) * V^T (n x n),
    // Then build pseudoinverse => V diag(...)^+ U^T.

    // Make local copy of *this => B (m x n) which we'll transform
    Matrix B(m, n, 0.0);
    for(unsigned int i = 0; i < m; i++){
        for(unsigned int j = 0; j < n; j++){
            B(i,j) = (*this)(i,j);
        }
    }

    // U: (m x m), initially identity
    Matrix U(m); // By your spec, Matrix(m) => identity(m x m)
    // V: (n x n), initially identity
    Matrix V(n);

    // We'll keep main diagonal in diag[], super-diagonal in super[].
    unsigned int mn = (m < n ? m : n);
    double* diag  = new double[mn];
    double* super = new double[mn];
    for(unsigned int i=0; i<mn; i++){
        diag[i]  = 0.0;
        super[i] = 0.0;
    }

    // ----------------------------------------------------
    // 1) HOUSEHOLDER BIDIAGONALIZATION (adapted from *this code)

    unsigned int nct = (m - 1 < n ? m - 1 : n);
    unsigned int nrt = (n - 2 < m ? n - 2 : m);

    for(unsigned int i = 0; i < (nct > mn ? mn : nct); i++){
        //---------------------
        // LEFT HOUSEHOLDER
        double normx = 0.0;
        for(unsigned int r=i; r<m; r++){
            double val = B(r,i);
            normx += val*val;
        }
        normx = std::sqrt(normx);

        if(normx < 1e-30){
            diag[i] = 0.0;
        } else {
            if(B(i,i) > 0.0){
                normx = -normx;
            }
            diag[i] = normx;
            for(unsigned int r=i; r<m; r++){
                B(r,i) /= -normx;
            }
            B(i,i) += 1.0;

            // apply to the rest of the columns
            for(unsigned int c = i+1; c < n; c++){
                double s = 0.0;
                for(unsigned int r = i; r < m; r++){
                    s += B(r,i)*B(r,c);
                }
                s = -s / B(i,i);
                for(unsigned int r=i; r<m; r++){
                    B(r,c) += s*B(r,i);
                }
            }
        }

        // Accumulate in U
        if(normx > 1e-30){
            for(unsigned int c=0; c<m; c++){
                double s=0.0;
                for(unsigned int r=i; r<m; r++){
                    s += B(r,i)*U(r,c);
                }
                s = -s / B(i,i);
                for(unsigned int r=i; r<m; r++){
                    U(r,c) += s*B(r,i);
                }
            }
        }

        diag[i] = -normx;

        //---------------------
        // RIGHT HOUSEHOLDER
        if(i < n-1){
            double normy = 0.0;
            for(unsigned int c=i+1; c<n; c++){
                double val = B(i,c);
                normy += val*val;
            }
            normy = std::sqrt(normy);

            if(normy < 1e-30){
                super[i] = 0.0;
            } else {
                if(B(i,i+1) > 0.0){
                    normy = -normy;
                }
                super[i] = normy;
                for(unsigned int c = i+1; c<n; c++){
                    B(i,c) /= -normy;
                }
                B(i,i+1) += 1.0;

                // apply to rows i+1..m-1
                for(unsigned int rr=i+1; rr<m; rr++){
                    double s=0.0;
                    for(unsigned int c2=i+1; c2<n; c2++){
                        s += B(i,c2)*B(rr,c2);
                    }
                    s = -s / B(i,i+1);
                    for(unsigned int c2=i+1; c2<n; c2++){
                        B(rr,c2) += s*B(i,c2);
                    }
                }
            }

            // accumulate in V
            if(normy > 1e-30){
                for(unsigned int rr=0; rr<n; rr++){
                    double s=0.0;
                    for(unsigned int c2=i+1; c2<n; c2++){
                        s += B(i,c2)*V(rr,c2);
                    }
                    s = -s / B(i,i+1);
                    for(unsigned int c2=i+1; c2<n; c2++){
                        V(rr,c2) += s*B(i,c2);
                    }
                }
            }
        }
    }

    // fill diag[] and super[]
    for(unsigned int i=0; i<mn; i++){
        diag[i] = B(i,i);
        if(i < mn-1){
            super[i] = ((i+1<n)? B(i,i+1) : 0.0);
        }
    }

    // ----------------------------------------------------
    // 2) Golub-Kahan Bidiagonal QR iteration

    auto fabsd = [&](double x){return (x<0)?-x:x;};

    for(int iterCount=0; iterCount<maxIter; iterCount++){
        // zero out super[i] if it's small
        int numZero=0;
        for(unsigned int i=0; i<mn-1; i++){
            double thresh = rankEps*(fabsd(diag[i]) + fabsd(diag[i+1]));
            if(std::fabs(super[i]) < thresh){
                super[i] = 0.0;
                numZero++;
            }
        }
        if(numZero == (int)(mn-1)){
            break; // done
        }

        // find sub-block
        unsigned int start=0, end=mn-1;
        while(start<mn-1){
            if(std::fabs(super[start]) < 1e-30){
                start++;
            } else {
                break;
            }
        }
        while(end>0){
            if(std::fabs(super[end-1])<1e-30){
                end--;
            } else {
                break;
            }
        }
        if(start>=end-1){
            continue;
        }

        // SHIFT
        double alpha = diag[end-1];
        double beta = 0.0;
        if((end-2) < mn){
            beta = super[end-2];
        }
        double shift=0.0;
        if(end>=2){
            double dd = diag[end-2];
            double bb = (end>=3? super[end-3]:0.0);
            double t = 0.5*(dd - alpha);
            double sq = t*t + bb*bb;
            double sign = (t>=0?1.0:-1.0);
            shift = alpha - sign*std::sqrt(sq);
        } else {
            shift = alpha;
        }

        double x = diag[start] - shift;
        double z = super[start];

        // Givens sweeps
        for(unsigned int k=start; k<end-1; k++){
            // RIGHT GIVENS
            double c=0.0, s=0.0;
            {
                if(std::fabs(z)<1e-30){
                    c=1.0; s=0.0;
                } else {
                    double r = std::sqrt(x*x + z*z);
                    c= x/r; 
                    s= z/r;
                }
            }
            double mk  = diag[k];
            double mk1 = diag[k+1];
            double sk  = super[k];
            diag[k]   = c*mk - s*sk;
            super[k]  = s*mk + c*sk;
            diag[k+1] = c*mk1;

            // apply to V
            for(unsigned int row=0; row<n; row++){
                double vrk  = V(row,k);
                double vrk1 = V(row,k+1);
                V(row,k)   = c*vrk - s*vrk1;
                V(row,k+1) = s*vrk + c*vrk1;
            }

            if(k<end-2){
                // LEFT GIVENS
                x = super[k];
                z = diag[k+1];
                double r = std::sqrt(x*x + z*z);
                if(r<1e-30){
                    c=1.0; s=0.0;
                } else {
                    c=x/r; s=z/r;
                }
                super[k] = r;

                double dk1 = diag[k+1];
                double sk1 = super[k+1];
                diag[k+1]  = c*dk1 - s*sk1;
                super[k+1] = s*dk1 + c*sk1;

                // apply to U, columns (k+1, k+2) if in range
                if((k+2) < m){
                    for(unsigned int row=0; row<m; row++){
                        double uk1= U(row,k+1);
                        double uk2= U(row,k+2);
                        U(row,k+1) = c*uk1 - s*uk2;
                        U(row,k+2) = s*uk1 + c*uk2;
                    }
                }
                x= diag[k+1];
                z= super[k+1];
            }
        }
    }

    // Now diag[] is ~ singular values (some might be negative)
    for(unsigned int i=0; i<mn; i++){
        if(diag[i]<0.0){
            diag[i] = -diag[i];
            // flip sign in U's column i
            for(unsigned int r=0; r<m; r++){
                U(r,i) = -U(r,i);
            }
        }
    }

    // sort descending
    for(unsigned int i=0; i<mn; i++){
        double bestVal=diag[i];
        unsigned int bestPos=i;
        for(unsigned int j=i+1; j<mn; j++){
            if(diag[j]>bestVal){
                bestVal=diag[j];
                bestPos=j;
            }
        }
        if(bestPos!=i){
            double tmp=diag[i];
            diag[i]=diag[bestPos];
            diag[bestPos]=tmp;
            // swap columns i,bestPos in U
            for(unsigned int r=0; r<m; r++){
                double t2= U(r,i);
                U(r,i)= U(r,bestPos);
                U(r,bestPos)= t2;
            }
            // swap columns i,bestPos in V
            for(unsigned int r=0; r<n; r++){
                double t2= V(r,i);
                V(r,i)= V(r,bestPos);
                V(r,bestPos)= t2;
            }
        }
    }

    for (int iterCount = 0; iterCount < maxIter; ++iterCount) {
        // Detect negligible superdiagonals
        int numZero = 0;
        for (unsigned int i = 0; i < mn - 1; i++) {
            double thresh = rankEps * (std::fabs(diag[i]) + std::fabs(diag[i + 1]));
            if (std::fabs(super[i]) < thresh) {
                super[i] = 0.0;
                numZero++;
            }
        }
        if (numZero == (int)(mn - 1)) break; // fully diagonal
    
        // Find active sub-block
        unsigned int start = 0, end = mn - 1;
        while (start < mn - 1 && std::fabs(super[start]) < 1e-30) start++;
        while (end > 0 && std::fabs(super[end - 1]) < 1e-30) end--;
        if (start >= end - 1) continue;
    
        // Use bottom 2x2 for Wilkinson shift
        double shift = 0.0;
        if (end >= 2) {
            double a = diag[end - 2], b = super[end - 2], c = diag[end - 1];
            double delta = (a - c) * 0.5;
            double t = std::sqrt(delta * delta + b * b);
            shift = c - (b * b) / (delta + ((delta >= 0) ? t : -t));
        } else {
            shift = diag[end - 1];
        }
    
        double x = diag[start] - shift;
        double z = super[start];
    
        for (unsigned int k = start; k < end - 1; k++) {
            // Right Givens (affects V)
            double r = std::sqrt(x * x + z * z);
            double c = (r < 1e-30) ? 1.0 : x / r;
            double s = (r < 1e-30) ? 0.0 : z / r;
    
            // Apply to bidiagonal
            double t1 = diag[k], t2 = super[k];
            diag[k]   = c * t1 - s * t2;
            super[k]  = s * t1 + c * t2;
            diag[k+1] = c * diag[k+1];
    
            // Apply to V (columns k, k+1)
            for (unsigned int row = 0; row < n; row++) {
                double vk = V(row, k), vk1 = V(row, k + 1);
                V(row, k)     = c * vk - s * vk1;
                V(row, k + 1) = s * vk + c * vk1;
            }
    
            // Left Givens (affects U)
            if (k < end - 2) {
                x = super[k];
                z = diag[k + 1];
                r = std::sqrt(x * x + z * z);
                c = (r < 1e-30) ? 1.0 : x / r;
                s = (r < 1e-30) ? 0.0 : z / r;
    
                super[k]   = r;
                diag[k + 1] = c * diag[k + 1] - s * super[k + 1];
                super[k + 1] = s * diag[k + 1] + c * super[k + 1];
    
                // Apply to U (columns k+1, k+2)
                if (k + 2 < m) {
                    for (unsigned int row = 0; row < m; row++) {
                        double uk1 = U(row, k + 1), uk2 = U(row, k + 2);
                        U(row, k + 1) = c * uk1 - s * uk2;
                        U(row, k + 2) = s * uk1 + c * uk2;
                    }
                }
    
                x = diag[k + 1];
                z = super[k + 1];
            }
        }
    }

    // Build Sigma (m x n)
    Matrix Sigma(m, n, 0.0);
    for(unsigned int i=0; i<m && i<n; i++){
        Sigma(i,i) = diag[i];
    }

    // Build Sigma^+ => (n x m)
    Matrix SigmaPlus(n, m, 0.0);
    for(unsigned int i=0; i<mn; i++){
        double val = diag[i];
        if(val>rankEps){
            SigmaPlus(i,i)= 1.0/val;
        } else {
            SigmaPlus(i,i)= 0.0;
        }
    }

    // Mtemp = V*(n x n)*SigmaPlus(n x m) => (n x m)
    Matrix Mtemp = V.multiply(SigmaPlus);

    // U^T => (m x m)
    Matrix U_T(m, m, 0.0);
    for(unsigned int r=0; r<m; r++){
        for(unsigned int c=0; c<m; c++){
            U_T(r,c)= U(c,r);
        }
    }

    // Aplus => (n x m) = (n x m)*(m x m) => (n x m)
    Matrix Aplus = Mtemp.multiply(U_T);

    delete[] diag;
    delete[] super;

    return Aplus;
}

Matrix Matrix::pseudoInverse() const {
    double tol = 1e-8;
    // Check if matrix is non-empty
    if (rows == 0 || cols == 0) {
        // std::cerr << "[ERROR] pseudoInverse: Empty matrix.\n";
        return Matrix(0, 0, 0);
    }

    // std::cout << "[INFO] Starting pseudoinverse computation...\n";

    // Step 1: Transpose of A
    Matrix A_T = this->transpose();
    // std::cout << "[INFO] Transposed matrix A_T computed.\n";

    // Step 2: AᵗA (n x n)
    Matrix AtA = A_T.multiply(*this);
    // std::cout << "[INFO] Computed AtA = Aᵗ * A\n";

    // Step 3: Regularization for invertibility (Tikhonov method)
    for (unsigned int i = 0; i < AtA.rows; ++i) {
        AtA(i, i) += tol;
    }
    // std::cout << "[INFO] Applied Tikhonov regularization to AtA.\n";

    // Step 4: Attempt inversion of regularized AtA
    Matrix AtA_inv = AtA.luInverse();
    if (AtA_inv.rows == 0 || AtA_inv.cols == 0) {
        // std::cerr << "[WARNING] AtA inversion failed. Trying alternate path using A * Aᵗ...\n";

        // If AtA fails, try A * Aᵗ
        Matrix AAt = this->multiply(A_T);
        for (unsigned int i = 0; i < AAt.rows; ++i) {
            AAt(i, i) += tol;
        }

        Matrix AAt_inv = AAt.luInverse();
        if (AAt_inv.rows == 0 || AAt_inv.cols == 0) {
            std::cerr << "[ERROR] Both AtA and AAt are singular. Pseudoinverse failed.\n";
            return Matrix(0, 0, 0);
        }

        // Compute right pseudoinverse: Aᵗ * (A * Aᵗ)⁻¹
        // std::cout << "[INFO] Successfully computed pseudoinverse using Aᵗ * (A * Aᵗ)⁻¹.\n";
        return A_T.multiply(AAt_inv);
    }

    // Step 5: Compute pseudoinverse: (AᵗA + λI)⁻¹ * Aᵗ
    // std::cout << "[INFO] Successfully computed pseudoinverse using (AᵗA + λI)⁻¹ * Aᵗ.\n";
    return AtA_inv.multiply(A_T);
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
