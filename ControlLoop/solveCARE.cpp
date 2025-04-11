#include "Matrix.h"
#include <iostream>

extern "C" {
    // SELECT function: tells LAPACK which eigenvalues to sort to the top-left
    int select_neg_real(double* wr, double* wi) {
        return (*wr < 0.0); // Select eigenvalues with negative real part
    }

    // LAPACK prototype
    void dgees_(
        char* jobvs, char* sort, int (*select)(double*, double*),
        int* n, double* a, int* lda, int* sdim,
        double* wr, double* wi, double* vs, int* ldvs,
        double* work, int* lwork, int* bwork, int* info
    );
    void dgetrf_(int* m, int* n, double* a, int* lda, int* ipiv, int* info);
    void dgetri_(int* n, double* a, int* lda, int* ipiv, double* work, int* lwork, int* info);
}


bool lapackInverse(Matrix& A) {
    int n = A.getRows();
    int lda = n;
    int info;

    double* data = new double[n * n];
    A.toArray(data, true);  // column-major layout

    int* ipiv = new int[n];
    dgetrf_(&n, &n, data, &lda, ipiv, &info);
    if (info != 0) {
        std::cerr << "[lapackInverse] LU factorization failed. INFO = " << info << "\n";
        return false;
    }

    // Query optimal workspace size
    int lwork = -1;
    double work_size;
    dgetri_(&n, data, &lda, ipiv, &work_size, &lwork, &info);
    lwork = static_cast<int>(work_size);
    double* work = new double[lwork];

    dgetri_(&n, data, &lda, ipiv, work, &lwork, &info);
    if (info != 0) {
        std::cerr << "[lapackInverse] Matrix inversion failed. INFO = " << info << "\n";
        return false;
    }

    // Load result back into A
    A = Matrix::fromArray(n, n, data, true);

    delete[] data;
    delete[] ipiv;
    delete[] work;

    return true;
}

Matrix solveCARE(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R) {
    const int n = A.getRows();
    const int m = B.getCols();
    int dim = 2 * n;

    // std::cout << "\n[solveCARE] === Begin CARE solver ===\n";
    // std::cout << "[solveCARE] Matrix sizes: n = " << n << ", m = " << m << "\n";

    if (A.getRows() != A.getCols()) throw std::runtime_error("[solveCARE] Matrix A must be square.");
    if (Q.getRows() != n || Q.getCols() != n) throw std::runtime_error("[solveCARE] Q must be " + std::to_string(n) + "x" + std::to_string(n));
    if (B.getRows() != n) throw std::runtime_error("[solveCARE] B must have " + std::to_string(n) + " rows.");
    if (R.getRows() != m || R.getCols() != m) throw std::runtime_error("[solveCARE] R must be " + std::to_string(m) + "x" + std::to_string(m));

    Matrix Rinv = R.pseudoInverse();
    Matrix BRBt = B.multiply(Rinv).multiply(B.transpose());

    Matrix H(dim, dim, 0.0);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            H(i, j) = A(i, j);
            H(i, j + n) = -BRBt(i, j);
            H(i + n, j) = -Q(i, j);
            H(i + n, j + n) = -A(j, i);
        }

    // std::cout << "[solveCARE] Allocating Schur workspace...\n";

    double* H_data = new double[dim * dim];
    H.toArray(H_data, true);  // Fill in column-major
    // std::cout << "[DEBUG] H_data @ " << static_cast<void*>(H_data) << ", size = " << (dim * dim) << "\n";

    double* vs = new double[dim * dim]();
    double* wr = new double[dim]();
    double* wi = new double[dim]();
    int* bwork = new int[dim]; 
    int sdim = 0, info = 0;
    double work_query = 0.0;
    int lwork = -1;

    char jobvs = 'V';
    char sort = 'S';

    // std::cout << "[solveCARE] Querying optimal lwork...\n";
    dgees_(&jobvs, &sort, select_neg_real, &dim, H_data, &dim, &sdim,
           wr, wi, vs, &dim, &work_query, &lwork, bwork, &info);

    if (info != 0) {
        std::cerr << "[solveCARE] LAPACK dgees_ query failed. INFO = " << info << "\n";
        if (info != 0 || sdim != n) {
            delete[] H_data;
            delete[] vs;
            delete[] wr;
            delete[] wi;
            delete[] bwork;
            throw std::runtime_error("CARE failed...");
        }
    }

    lwork = static_cast<int>(work_query);
    double* work = new double[lwork]();
    // std::cout << "[DEBUG] work buffer @ " << static_cast<void*>(work) << ", size = " << lwork << "\n";

    // std::cout << "[solveCARE] Running LAPACK Schur decomposition...\n";
    int (*select_fn)(double*, double*) = select_neg_real;
    dgees_(&jobvs, &sort, select_fn, &dim, H_data, &dim, &sdim,
           wr, wi, vs, &dim, work, &lwork, bwork, &info);

    if (info != 0 || sdim != n) {
        if (info != 0 || sdim != n) {
            delete[] H_data;
            delete[] vs;
            delete[] wr;
            delete[] wi;
            delete[] bwork;
            delete[] work;
            throw std::runtime_error("CARE failed...");
        }
    }

    // std::cout << "[solveCARE] Schur decomposition OK. Copying stable subspace...\n";
    Matrix Z(dim, n, 0.0);
    for (int col = 0; col < n; ++col)
        for (int row = 0; row < dim; ++row) {
            int flat_index = col * dim + row;
            if (flat_index >= dim * dim) {
                std::cerr << "[FATAL] Out-of-bounds read on vs[" << flat_index << "]\n";
                throw std::runtime_error("Memory read out of bounds on vs[]");
            }
            Z(row, col) = vs[flat_index];
        }

    Matrix U11(n, n, 0), U21(n, n, 0);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            U11(i, j) = Z(i, j);
            U21(i, j) = Z(i + n, j);
        }

    Matrix U11inv = U11.pseudoInverse();
    Matrix X = U21.multiply(U11inv);

    // Free everything after successful result
    delete[] H_data;
    delete[] vs;
    delete[] wr;
    delete[] wi;
    delete[] bwork;
    delete[] work;

    return X;
}
