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
        double* work, int* lwork, bool* bwork, int* info
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
    int n = A.getRows();
    int m = B.getCols();

    // std::cout << "\n[solveCARE] === Begin CARE solver ===\n";
    // std::cout << "[solveCARE] Matrix sizes: n = " << n << ", m = " << m << "\n";

    // === Step 1: Compute S = B * R⁻¹ * Bᵗ ===
    // std::cout << "[solveCARE] Inverting R...\n";
    Matrix Rinv = R.pseudoInverse();
    // std::cout << "[solveCARE] R⁻¹:\n"; Rinv.print();

    // std::cout << "[solveCARE] Computing B * R⁻¹ * Bᵗ...\n";
    Matrix BRBt = B.multiply(Rinv).multiply(B.transpose());
    //std::cout << "[solveCARE] B R⁻¹ Bᵗ:\n"; BRBt.print();

    // === Step 2: Form the Hamiltonian matrix H ===
    // std::cout << "[solveCARE] Building Hamiltonian matrix H...\n";
    Matrix H(2 * n, 2 * n, 0.0);

    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            H(i, j) = A(i, j);                    // A
            H(i, j + n) = -BRBt(i, j);            // -B R⁻¹ Bᵗ
            H(i + n, j) = -Q(i, j);               // -Q
            H(i + n, j + n) = -A(j, i);           // -Aᵗ
        }

    // std::cout << "[solveCARE] Hamiltonian H:\n";
    // H.print();

    // === Step 3: Perform Schur decomposition with sorting ===
    //std::cout << "[solveCARE] Performing Schur decomposition with eigenvalue sorting...\n";
    int dim = 2 * n;
    double* H_data = new double[dim * dim];
    H.toArray(H_data, true); // Column-major for LAPACK

    double* vs = new double[dim * dim];
    double* wr = new double[dim];
    double* wi = new double[dim];
    bool* bwork = new bool[dim];
    int sdim, info;

    char jobvs = 'V';  // Compute Schur vectors
    char sort = 'N';   // Sort eigenvalues
    int lwork = -1;
    double work_query;

    

    dgees_(&jobvs, &sort, select_neg_real, &dim, H_data, &dim, &sdim,
           wr, wi, vs, &dim, &work_query, &lwork, bwork, &info);

    lwork = static_cast<int>(work_query);
    double* work = new double[lwork];

    // std::cout << "[Schur] dim = " << dim << ", lwork = " << lwork << std::endl;
    if (!H_data || !vs || !wr || !wi || !bwork || !work) {
        std::cerr << "[ERROR] One or more arrays are null!" << std::endl;
        exit(1);
    }

    dgees_(&jobvs, &sort, select_neg_real, &dim, H_data, &dim, &sdim,
           wr, wi, vs, &dim, work, &lwork, bwork, &info);


    if (info != 0) {
        std::cerr << "[solveCARE] ERROR: Schur decomposition failed. INFO = " << info << "\n";
        exit(1);
    }
    
    if (sort == 'S' && sdim != n) {
        std::cerr << "[solveCARE] ERROR: Unexpected number of stable eigenvalues. SDIM = "
                << sdim << " (expected " << n << ")\n";
        exit(1);
    }


    // std::cout << "[solveCARE] Schur decomposition successful. SDIM = " << sdim << "\n";
    //std::cout << "[solveCARE] Real parts of eigenvalues (wr): ";
    // for (int i = 0; i < 2 * n; ++i) std::cout << wr[i] << " ";
    // std::cout << "\n";

    // std::cout << "[solveCARE] Imag parts of eigenvalues (wi): ";
    // for (int i = 0; i < 2 * n; ++i) std::cout << wi[i] << " ";
    // std::cout << "\n";

    // === Step 4: Extract X = U21 * U11⁻¹ ===
    // std::cout << "[solveCARE] Extracting invariant subspace from Schur vectors...\n";
    Matrix Z = Matrix::fromArray(dim, dim, vs, true);
    Matrix U11(n, n, 0), U21(n, n, 0);

    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            U11(i, j) = Z(i, j);       // Top part
            U21(i, j) = Z(i + n, j);   // Bottom part
        }

    // std::cout << "[solveCARE] U11 (upper subspace):\n"; U11.print();
    // std::cout << "[solveCARE] U21 (lower subspace):\n"; U21.print();

    // std::cout << "[solveCARE] Inverting U11...\n";
    Matrix U11inv = U11.pseudoInverse();
    
    // std::cout << "[solveCARE] U11⁻¹:\n"; U11inv.print();

    Matrix X = U21.multiply(U11inv);
    // std::cout << "[solveCARE] Solution X = U21 * U11⁻¹:\n"; X.print();

    // === Cleanup ===
    delete[] H_data;
    delete[] vs;
    delete[] wr;
    delete[] wi;
    delete[] bwork;
    delete[] work;

    // std::cout << "[solveCARE] === Done ===\n";
    return X;
}