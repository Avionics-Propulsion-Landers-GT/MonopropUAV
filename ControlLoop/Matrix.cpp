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

double innerProduct(const Matrix& v1, const Matrix& v2) {
    if (v1.getCols() != 1 || v2.getCols() != 1 || v1.getRows() != v2.getRows()) {
        throw std::runtime_error("innerProduct: Inputs must be column vectors of same length.");
    }
    double sum = 0.0;
    for (unsigned int i = 0; i < v1.getRows(); ++i) {
        sum += v1(i, 0) * v2(i, 0);
    }
    return sum;
}

double vectorNorm(const Matrix& v) {
    return std::sqrt(innerProduct(v, v));
}

Matrix reshapeVectorToMatrix(const Matrix& v, unsigned int n) {
    if (v.getCols() != 1 || v.getRows() != n * n)
        throw std::runtime_error("reshapeVectorToMatrix: vector size must be n^2 × 1.");

    Matrix M(n, n, 0.0);
    for (unsigned int i = 0; i < n; ++i) {
        for (unsigned int j = 0; j < n; ++j) {
            M(i, j) = v(i * n + j, 0);  // Row-major → tweak if you want column-major
        }
    }
    return M;
}
Matrix solveCARE_FixedPoint(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R, const double& alpha) {
    unsigned int n = A.getRows();
    if (A.getCols() != n || B.getRows() != n || Q.getRows() != n || Q.getCols() != n ||
        R.getRows() != B.getCols() || R.getCols() != B.getCols()) {
        throw std::runtime_error("solveCARE_FixedPoint: dimension mismatch.");
    }

    Matrix P = Q;  // Initial guess
    const int maxIter = 100;
    const double tol = 1e-5;
    Matrix At = A.transpose();
    Matrix Bt = B.transpose();
    Matrix Pk = P;

    Matrix alphaMat(n);
    alphaMat = alphaMat.multiply(alpha);

    for (int iter = 0; iter < maxIter; ++iter) {
        Matrix term1 = At.multiply(Pk);
        Matrix term2 = Pk.multiply(A);
        Matrix Ri = R.luInverse();
        Matrix BRiBt = B.multiply(Ri).multiply(Bt);
        Matrix term3 = (Pk.multiply(BRiBt).multiply(Pk)).multiply(-1);
        Matrix residual = (term1.add(term2).add(term3).add(Q)).multiply(-1);
        bool breakOut = false;

    

        Matrix dP(n, n, 0.0);
        for (unsigned int i = 0; i < n; ++i) {
            for (unsigned int j = 0; j < n; ++j) {
                double scale = std::max(1.0, std::min(10.0, std::abs(Pk(i, j))));
                double raw_step = residual(i, j) / scale;

                double drift = std::abs(raw_step * alphaMat(i, j));
                double rel_drift = drift / (std::abs(Pk(i,j)) + 1e-6);
                std::cout << "Drift: " << rel_drift << "\n";
                double abs_resid = std::abs(residual(i, j));

                // Boost α if it's stuck
                if (abs_resid > 0.05 && rel_drift < 0.5) {
                    alphaMat(i, j) *= 1.2;
                    if (alphaMat(i, j) > 50.0)
                        alphaMat(i, j) = 50.0;
                }

                // Reduce α if already converging
                if (abs_resid < 1e-10) {
                    alphaMat(i, j) *= 0.95;
                    if (alphaMat(i, j) < 1e-3)
                        alphaMat(i, j) = 1e-3;
                }

                // if (Pk(i,j) > 10000) {
                //     breakOut = true;
                // }

                dP(i, j) = raw_step * alphaMat(i, j);
            }
        }

        Matrix Pk1 = Pk.add(dP);
        // Matrix Pk1 = Pk1.add(Pk1.transpose()).multiply(0.5);

        

        // Convergence check
        Matrix diff = residual;
        double err = diff.frobeniusNorm();

        std::cout << std::fixed << std::setprecision(10);
        std::cout << "[CARE-FixedPoint] Iter " << iter + 1 << ", residual = " << err << ", P: \n"; Pk1.print();
        std::cout << std::fixed << std::setprecision(5);
        std::cout << "[CARE-FixedPoint] Residual: \n"; diff.print();
        std::cout << "[CARE-FixedPoint] AlphaMat: \n"; alphaMat.print();


        Pk = Pk1;

        if ((err < tol) || breakOut) {
            std::cout << "[CARE] Converged in " << iter << " iterations. \n";
            break;
        }
        
    }

    return Pk;
}

Matrix solveCARE_diagonal(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R) {
    unsigned int n = A.getRows();
    Matrix P(n, n, 0.0);

    for (unsigned int i = 0; i < n; ++i) {
        double a = (B(i,i) * B(i,i)) / R(i,i);
        double b = -2.0 * A(i,i);
        double c = -Q(i,i);

        double discriminant = b*b - 4*a*c;
        if (discriminant < 0.0)
            throw std::runtime_error("CARE (scalar): negative discriminant — no real solution.");

        double P_i = (-b + std::sqrt(discriminant)) / (2*a);
        P(i,i) = P_i;
    }

    return P;
}



double Matrix::frobeniusNorm() const {
    double sum = 0.0;
    for (unsigned int i = 0; i < this->rows; ++i) {
        for (unsigned int j = 0; j < this->cols; ++j) {
            double val = (*this)(i, j);
            sum += val * val;
        }
    }
    return std::sqrt(sum);
}


Matrix Matrix::pseudoInverseJacobi(double rankEps, int maxIter) const
{


    // ----------------------------------------------------------------
    // 1) Setup:
    //    A is m x n with m >= n (tall).
    //    We'll compute a 'thin' SVD: A = U * Sigma * V^T,
    //    where U is (m x n), Sigma is (n x n), and V is (n x n).
    // ----------------------------------------------------------------
    unsigned int m = this->rows; // e.g. 12
    unsigned int n = this->cols; // e.g. 7
    bool flipped = false;
    if (m < n) {
        // std::cout << "[SVD] Flipping...\n";
        Matrix At = *this;
        Matrix At_orig = *this;
        Matrix A = At.transpose();
        Matrix A_orig = A_orig.transpose();
        flipped = true;

    } else {
        Matrix A = *this;
        Matrix A_orig = *this;
    }

    // std::cout << "[pseudoInverseJacobi] Building a THIN SVD for m x n = "
    //          << m << " x " << n << "\n";

    // Copy A into local memory, which we'll orthogonalize
    Matrix A = *this;
    Matrix A_orig = *this;

    // The right singular vectors (n x n), initially identity
    Matrix V(n, n, 0.0);
    for (unsigned int i = 0; i < n; ++i) {
        V(i, i) = 1.0;
    }

    // ----------------------------------------------------------------
    // 2) One-Sided Jacobi on columns of A (m x n), accumulate in V (n x n).
    //    We'll attempt to zero out dot-products col p, col q.
    //
    //    After convergence:
    //       A(:,j) ~ sigma_j * u_j (each col is an S.V. scaled by sigma).
    //       V(:,j) = v_j (the right singular vector).
    // ----------------------------------------------------------------
    for (int iter = 0; iter < maxIter; ++iter)
    {
        bool changed = false;

        // std::cout << "\n[Jacobi iteration #" << iter << "]\n";
        // Print the norms of columns for debugging
        for (unsigned int col = 0; col < n; ++col) {
            double normCol = 0.0;
            for (unsigned int row = 0; row < m; ++row) {
                double val = A(row, col);
                normCol += val*val;
            }
            // std::cout << "  Norm of A.col[" << col << "] = " << std::sqrt(normCol) << "\n";
        }

        // Sweep over all column pairs (p,q)
        for (unsigned int p = 0; p < n - 1; ++p) {
            for (unsigned int q = p + 1; q < n; ++q) {

                // Compute:
                //  app = ||col p||^2, aqq = ||col q||^2, apq = dot(col p, col q)
                double app = 0.0, aqq = 0.0, apq = 0.0;
                for (unsigned int i = 0; i < m; ++i) {
                    double aip = A(i, p);
                    double aiq = A(i, q);
                    app += aip * aip;
                    aqq += aiq * aiq;
                    apq += aip * aiq;
                }

                // Skip if columns p,q are nearly orthogonal or degenerate
                double eps_orth = 1e-12 * std::sqrt(app * aqq);
                if (std::fabs(apq) <= eps_orth || app < 1e-30 || aqq < 1e-30)
                    continue;

                changed = true;

                // Jacobi rotation to kill cross-term in columns p,q
                double tau = (aqq - app) / (2.0 * apq);
                double t   = ( (tau >= 0.0) ? 1.0 : -1.0 )
                             / ( std::fabs(tau) + std::sqrt(1.0 + tau*tau) );
                double c = 1.0 / std::sqrt(1.0 + t*t);
                double s = c * t;

                // Rotate columns p,q of A
                for (unsigned int i = 0; i < m; ++i) {
                    double aip = A(i, p);
                    double aiq = A(i, q);
                    A(i, p) = c*aip - s*aiq;
                    A(i, q) = s*aip + c*aiq;
                }

                // Accumulate the same rotation in V
                for (unsigned int i = 0; i < n; ++i) {
                    double vip = V(i, p);
                    double viq = V(i, q);
                    V(i, p) = c*vip - s*viq;
                    V(i, q) = s*vip + c*viq;
                }
            }
        }

        if (!changed) {
            // std::cout << "  No changes => Jacobi has converged.\n";
            break;
        }
    }

    // ----------------------------------------------------------------
    // 3) Now each column j of A is sigma_j * (some length-m vector).
    //    We'll measure sigma_j = ||A(:,j)|| and store it in an array.
    // ----------------------------------------------------------------
    // std::cout << "\n[Post-Jacobi] Checking final column norms...\n";
    Matrix sigma(n, 1, 0.0);
    for (unsigned int j = 0; j < n; ++j) {
        double sum = 0.0;
        for (unsigned int i = 0; i < m; ++i) {
            double val = A(i, j);
            sum += val*val;
        }
        sigma(j, 0) = std::sqrt(sum);
        // std::cout << "  sigma(" << j << ") = " << sigma(j,0) << "\n";
    }

    // ----------------------------------------------------------------
    // 4) Build U (m x n) by normalizing the columns of A:
    //      A(:,j) = sigma_j * U(:,j)
    // ----------------------------------------------------------------
    Matrix U(m, n, 0.0);
    for (unsigned int j = 0; j < n; ++j) {
        double sVal = sigma(j,0);
        if (sVal > rankEps) {
            // U(:,j) = A(:,j) / sVal
            for (unsigned int i = 0; i < m; ++i) {
                U(i, j) = A(i, j) / sVal;
            }
        } else {
            // If sigma_j is tiny, zero out that column
            for (unsigned int i = 0; i < m; ++i) {
                U(i, j) = 0.0;
            }
            // std::cout << "  [INFO] sigma(" << j << ") <= rankEps => zeroing out col " << j << " in U\n";
        }
    }

    // ----------------------------------------------------------------
    // 5) Build the (n x n) Sigma matrix. Since sigma is 1D, store on diagonal.
    // ----------------------------------------------------------------
    Matrix Sigma(n, n, 0.0);
    for (unsigned int j = 0; j < n; ++j) {
        double sVal = sigma(j,0);
        Sigma(j, j) = (sVal > rankEps) ? sVal : 0.0;
    }

    // ----------------------------------------------------------------
    // 6) Build the pseudoinverse: A^+ = V * Sigma^+ * U^T
    //    Sigma^+ is also n x n, but with 1/sigma_j on diagonal (where nonzero).
    // ----------------------------------------------------------------
    // std::cout << "\n--- Building SigmaPlus (n x n) ---\n";
    Matrix SigmaPlus(n, n, 0.0);
    for (unsigned int j = 0; j < n; ++j) {
        double sVal = Sigma(j, j);
        if (sVal > rankEps) {
            SigmaPlus(j, j) = 1.0 / sVal;
        }
    }

    // std::cout << "\n--- Computing A^+ = V * SigmaPlus * U^T ---\n";
    Matrix Aplus = V.multiply(SigmaPlus).multiply(U.transpose()); // (n x m) = (7 x 12) in your case

    // ----------------------------------------------------------------
    // 7) Debug printing
    // ----------------------------------------------------------------
    // std::cout << "\n[Thin SVD Results]\n";
    // std::cout << "U (m x n): " << U.getRows() << " x " << U.getCols() << "\n";
    // U.print();
    // std::cout << "\nV (n x n): " << V.getRows() << " x " << V.getCols() << "\n";
    // V.print();
    // std::cout << "\nSigma (n x n):\n";
    // Sigma.print();
    // std::cout << "\nSigmaPlus (n x n):\n";
    // SigmaPlus.print();

    // std::cout << "\nAplus: " << Aplus.getRows() << " x " << Aplus.getCols() << "\n";
    // Aplus.print();

    // ----------------------------------------------------------------
    // 8) Validate Penrose conditions
    // ----------------------------------------------------------------
    // std::cout << "\n--- Penrose Validation ---\n";

    Matrix BBp  = A_orig.multiply(Aplus);
    Matrix BpB  = Aplus.multiply(A_orig);
    Matrix BBpB = BBp.multiply(A_orig);
    Matrix BpBBp= BpB.multiply(Aplus);

    Matrix BBpT = BBp.transpose();
    Matrix BpBT = BpB.transpose();

    auto approxEqual = [&](const Matrix &X, const Matrix &Y, double tol = 1e-7) {
        if (X.getRows()!=Y.getRows() || X.getCols()!=Y.getCols()) return false;
        for (unsigned int r=0; r<X.getRows(); ++r)
            for (unsigned int c=0; c<X.getCols(); ++c)
                if (std::fabs(X(r,c) - Y(r,c)) > tol) return false;
        return true;
    };
    auto frobeniusError = [&](const Matrix &X, const Matrix &Y) {
        if (X.getRows()!=Y.getRows() || X.getCols()!=Y.getCols())
            return std::numeric_limits<double>::quiet_NaN();
        double accum=0.0;
        for (unsigned int r=0; r<X.getRows(); ++r)
            for (unsigned int c=0; c<X.getCols(); ++c){
                double diff = X(r,c)-Y(r,c);
                accum += diff*diff;
            }
        return std::sqrt(accum);
    };

    // Compare BBpB vs A_orig
    double maxAbsErr=0.0;
    if (BBpB.getRows()==A_orig.getRows() && BBpB.getCols()==A_orig.getCols()) {
        for (unsigned int i=0; i<A_orig.getRows(); i++){
            for (unsigned int j=0; j<A_orig.getCols(); j++){
                maxAbsErr = std::max(maxAbsErr,
                                     std::fabs(BBpB(i,j)-A_orig(i,j)));
            }
        }
    }

    double frErr = frobeniusError(BBpB, A_orig);

    // std::cout << "  B * B^+: " << BBp.getRows() << " x " << BBp.getCols() << "\n";
    // std::cout << "  B^+ * B: " << BpB.getRows() << " x " << BpB.getCols() << "\n";
    // std::cout << "\n[Penrose Conditions]\n";
    // std::cout << "1) B * B^+ * B ≈ B:   maxAbsErr=" << maxAbsErr
    //           << ", froErr=" << frErr
    //           << (approxEqual(BBpB, A_orig) ? "  ✅" : "  ❌") << "\n";

    // std::cout << "2) B^+ * B * B^+ ≈ B^+: "
    //           << (approxEqual(BpBBp, Aplus) ? "✅" : "❌") << "\n";
    // std::cout << "3) (B * B^+)^T ≈ B * B^+: "
    //           << (approxEqual(BBpT, BBp) ? "✅" : "❌") << "\n";
    // std::cout << "4) (B^+ * B)^T ≈ B^+ * B: "
    //           << (approxEqual(BpBT, BpB) ? "✅" : "❌") << "\n";

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

        for (unsigned int r = 0; r < n; ++r) {
            for (unsigned int c = 0; c < m; ++c) {
                C(r, i * m + c) = term(r, c);
            }
        }

        A_pow = this->multiply(A_pow); // A^(i+1)
    }

    return C;
}

bool Matrix::isControllable(const Matrix& B, double tol) const {
    Matrix C = this->controllabilityMatrix(B);
    unsigned int n = this->getRows();
    unsigned int rankC = C.rank(tol); // You can customize this internally

    return rankC == n;
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


Matrix Matrix::pseudoInverseAuto(double rankEps, int maxIter) const {
    unsigned int m = this->getRows();
    unsigned int n = this->getCols();

    // 1. Estimate numerical rank via column norms
    int significantCols = 0;
    for (unsigned int j = 0; j < n; ++j) {
        double colNorm = 0.0;
        for (unsigned int i = 0; i < m; ++i) {
            colNorm += (*this)(i, j) * (*this)(i, j);
        }
        if (std::sqrt(colNorm) > rankEps) {
            ++significantCols;
        }
    }

    // std::cout << "[pseudoInverseAuto] Estimated rank = " << significantCols << "\n";

    // 2. If rank > 1 → Jacobi SVD
    if (significantCols > 1) {
        // std::cout << "  → Using full Jacobi SVD pseudoinverse\n";
        return this->pseudoInverseJacobi(rankEps, maxIter);
    }

    // 3. If rank == 1 → use fast transpose formula
    // std::cout << "  → Using fast rank-1 pseudoinverse via transpose\n";
    Matrix Bt = this->transpose();
    double frobSq = 0.0;
    for (unsigned int i = 0; i < m; ++i) {
        for (unsigned int j = 0; j < n; ++j) {
            double val = (*this)(i, j);
            frobSq += val * val;
        }
    }

    if (frobSq < 1e-20) {
        std::cerr << "[WARNING] Matrix norm is too small, returning zero pseudoinverse\n";
        return Matrix(n, m, 0.0);  // zero fallback
    }

    Matrix B_pinv = Bt.multiply((1.0 / frobSq));  // scale each entry
    return B_pinv;
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
