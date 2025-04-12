#include "../CustomLinear/Matrix.h"
#include <iostream>

extern "C" {
    int select_neg_real(double* wr, double* wi) {
        return (*wr < 0.0); // Select eigenvalues with negative real part
    }
    void dgees_(
        char* jobvs, char* sort, int (*select)(double*, double*),
        int* n, double* a, int* lda, int* sdim,
        double* wr, double* wi, double* vs, int* ldvs,
        double* work, int* lwork, int* bwork, int* info
    );
    void dgetrf_(int* m, int* n, double* a, int* lda, int* ipiv, int* info);
    void dgetri_(int* n, double* a, int* lda, int* ipiv, double* work, int* lwork, int* info);
}

Matrix solveCARE(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R) {
    const int n = A.getRows();
    const int m = B.getCols();
    int dim = 2 * n;

    if (A.getRows() != A.getCols()) throw std::runtime_error("[solveCARE] Matrix A must be square.");
    if (Q.getRows() != n || Q.getCols() != n) throw std::runtime_error("[solveCARE] Q must be " + std::to_string(n) + "x" + std::to_string(n));
    if (B.getRows() != n) throw std::runtime_error("[solveCARE] B must have " + std::to_string(n) + " rows.");
    if (R.getRows() != m || R.getCols() != m) throw std::runtime_error("[solveCARE] R must be " + std::to_string(m) + "x" + std::to_string(m));

    Matrix Rinv = R.pseudoInverseJacobi(10e-12, 100);
    Matrix BRBt = B.multiply(Rinv).multiply(B.transpose());

    Matrix H(dim, dim, 0.0);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            H(i, j) = A(i, j);
            H(i, j + n) = -BRBt(i, j);
            H(i + n, j) = -Q(i, j);
            H(i + n, j + n) = -A(j, i);
        }

    double* H_data = new double[dim * dim];
    H.toArray(H_data, true);  // Fill in column-major

    double* vs = new double[dim * dim]();
    double* wr = new double[dim]();
    double* wi = new double[dim]();
    int* bwork = new int[dim]; 
    int sdim = 0, info = 0;
    double work_query = 0.0;
    int lwork = -1;
    char jobvs = 'V';
    char sort = 'S';

    dgees_(&jobvs, &sort, select_neg_real, &dim, H_data, &dim, &sdim,
           wr, wi, vs, &dim, &work_query, &lwork, bwork, &info);

    lwork = static_cast<int>(work_query);
    double* work = new double[lwork]();
    int (*select_fn)(double*, double*) = select_neg_real;
    dgees_(&jobvs, &sort, select_fn, &dim, H_data, &dim, &sdim,
           wr, wi, vs, &dim, work, &lwork, bwork, &info);

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

    Matrix U11inv = U11.pseudoInverseJacobi(1e-12, 100);
    Matrix X = U21.multiply(U11inv);

    delete[] H_data;
    delete[] vs;
    delete[] wr;
    delete[] wi;
    delete[] bwork;
    delete[] work;

    return X;
}
