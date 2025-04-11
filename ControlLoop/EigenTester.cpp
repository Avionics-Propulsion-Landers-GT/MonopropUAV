#include <iostream>
#include <vector>
#include <cmath>
#include "Matrix.h"

extern "C" {
    void dgeesx_(
        char* jobvs, char* sort, int (*select)(double*, double*), char* sense,
        int* n, double* a, int* lda, int* sdim,
        double* wr, double* wi, double* vs, int* ldvs,
        double* rconde, double* rcondv,
        double* work, int* lwork, int* iwork, int* liwork,
        bool* bwork, int* info
    );
}

int main() {
    const int n = 5;
    double A_data[n * n] = {
        4, 2, 1, 3, 0,
        0, 1, 4, 1, 2,
        3, 0, 2, 0, 1,
        2, 3, 0, 1, 0,
        1, 2, 3, 4, 5
    };

    std::cout << "Input matrix A:\n";
    Matrix A = Matrix::fromArray(n, n, A_data, /*colMajor=*/false);
    A.print();

    // LAPACK expects column-major
    double A_lapack[n * n];
    std::copy(A_data, A_data + n * n, A_lapack);

    // Output arrays
    double wr[n], wi[n], vs[n * n];
    int lda = n, ldvs = n, sdim, info;

    // Workspace query
    double work_query;
    int lwork = -1, iwork_query;
    int liwork = -1;
    bool bwork[n];

    char jobvs = 'V';  // Compute Schur vectors
    char sort = 'N';   // Don't sort eigenvalues
    char sense = 'N';  // Don't compute condition numbers

    dgeesx_(&jobvs, &sort, nullptr, &sense,
            (int*)&n, A_lapack, &lda, &sdim,
            wr, wi, vs, &ldvs, nullptr, nullptr,
            &work_query, &lwork, &iwork_query, &liwork,
            bwork, &info);

    // Allocate workspace
    lwork = (int)work_query;
    liwork = iwork_query;
    std::vector<double> work(lwork);
    std::vector<int> iwork(liwork);

    // Actual computation
    dgeesx_(&jobvs, &sort, nullptr, &sense,
            (int*)&n, A_lapack, &lda, &sdim,
            wr, wi, vs, &ldvs, nullptr, nullptr,
            work.data(), &lwork, iwork.data(), &liwork,
            bwork, &info);

    if (info != 0) {
        std::cerr << "DGEESX failed with INFO = " << info << "\n";
        return 1;
    }

    // Print Schur form (A is overwritten with T)
    std::cout << "\nSchur form T:\n";
    Matrix T = Matrix::fromArray(n, n, A_lapack, true);
    T.print();

    // Print Schur vectors
    std::cout << "\nOrthogonal matrix Q:\n";
    Matrix Q = Matrix::fromArray(n, n, vs, true);
    Q.print();

    // Verify: A ≈ Q * T * Qᵗ
    Matrix A_reconstructed = Q.multiply(T).multiply(Q.transpose());
    std::cout << "\nReconstructed A (Q*T*Qᵗ):\n";
    A_reconstructed.print();

    std::cout << "\n=== Schur decomposition test (via LAPACK) complete ===\n";
    return 0;
}
