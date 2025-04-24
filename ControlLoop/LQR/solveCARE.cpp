#include "../CustomLinear/Matrix.h"
#include <iostream>

#include "../CustomLinear/Matrix.h"
#include <iostream>
#include <numeric>
#include <vector>
#include <algorithm>

extern "C" {
    // Eigenvalue selector for Schur form
    extern "C" int select_neg_real(double* wr, double* wi) {
        return (*wr < 0.0) ? 0xFFFFFFFF : 0x00000000;
    }
    
    // Schur decomposition (real, double precision)
    void dgees_(
        char* jobvs, char* sort, int (*select)(double*, double*),
        int* n, double* a, int* lda, int* sdim,
        double* wr, double* wi, double* vs, int* ldvs,
        double* work, int* lwork, int* bwork, int* info
    );

    void dgesv_(
        int* n, int* nrhs, double* a, int* lda,
        int* ipiv, double* b, int* ldb, int* info
    );
}

int chk() {
    const int n = 2;
    double A[n * n] = {0.0, 1.0, -2.0, -3.0};  // [ [0, 1], [-2, -3] ]
    double wr[n], wi[n], vs[n * n];
    double work_query;
    int sdim = 0, info = 0, lwork = -1;
    int bwork[n];

    char jobvs = 'V';
    char sort = 'S';

    // Workspace query
    dgees_(&jobvs, &sort, select_neg_real, (int*)&n, A, (int*)&n, &sdim,
           wr, wi, vs, (int*)&n, &work_query, &lwork, bwork, &info);

    lwork = (int)work_query;
    double* work = new double[lwork]();

    // Run the real thing
    dgees_(&jobvs, &sort, select_neg_real, (int*)&n, A, (int*)&n, &sdim,
           wr, wi, vs, (int*)&n, work, &lwork, bwork, &info);

    std::cout << "\n[TEST] Info: " << info << ", sdim = " << sdim << "\n";

    for (int i = 0; i < n; ++i)
        std::cout << "Eigenvalue: " << wr[i] << " + " << wi[i] << "i\n";

    delete[] work;
    return 0;
}

double frobeniusNorm2(const Matrix& A) {
    double sum = 0.0;
    for (unsigned int i = 0; i < A.getRows(); ++i) {
        for (unsigned int j = 0; j < A.getCols(); ++j) {
            double val = A(i, j);
            sum += val * val;
        }
    }
    return std::sqrt(sum);
}


std::vector<Matrix> solveCARE(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R) {
    // int rtv = chk();
    // std::cout << "rtv: " << rtv << std::endl;
    const int n = A.getRows();
    const int m = B.getCols();
    int dim = 2 * n;

    if (A.getRows() != A.getCols()) throw std::runtime_error("[solveCARE] Matrix A must be square.");
    if (Q.getRows() != n || Q.getCols() != n) throw std::runtime_error("[solveCARE] Q must be " + std::to_string(n) + "x" + std::to_string(n));
    if (B.getRows() != n) throw std::runtime_error("[solveCARE] B must have " + std::to_string(n) + " rows.");
    if (R.getRows() != m || R.getCols() != m) throw std::runtime_error("[solveCARE] R must be " + std::to_string(m) + "x" + std::to_string(m));

    Matrix Rinv = R.pseudoInverseJacobi(1e-15, 100);
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

   // std::cout << "\n H:\n "; H.print();
    

    lwork = static_cast<int>(work_query);
    double* work = new double[lwork]();
    int (*select_fn)(double*, double*) = select_neg_real;
    H.toArray(H_data, true); 

    dgees_(&jobvs, &sort, select_fn, &dim, H_data, &dim, &sdim,
           wr, wi, vs, &dim, work, &lwork, bwork, &info);

    // for (int i = 0; i < dim; ++i) {
    //     std::cout << "[eig] real = " << wr[i] << ", imag = " << wi[i] << "\n";
    // }

    std::cout << "[CARE] Expected " << n << " stable eigenvalues, got " << sdim << std::endl;

    // Build sorted indices based on real and imaginary parts
    std::vector<int> indices(n);
    std::iota(indices.begin(), indices.end(), 0);

    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        if (std::abs(wr[a] - wr[b]) > 1e-8) return wr[a] < wr[b];  // sort by real part
        return wi[a] < wi[b];  // tiebreak: imag part
    });

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

    Z = Z.reorderColumns(indices);

    Matrix U11(n, n, 0), U21(n, n, 0);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            U11(i, j) = Z(i, j);
            U21(i, j) = Z(i + n, j);
        }

    // std::cout << "\nU11fn:" << frobeniusNorm2(U11) << std::endl;
    // std::cout << "\nU21fn:" << frobeniusNorm2(U21) << std::endl;

    // -- Regularization strength --
    double lambda = 1e-8;

    // -- Sizes --
    int n_reg = U11.getRows();
    int k_rhs = U21.getCols();

    // -- Step 1: Compute AᵗA = U11ᵗ * U11 --
    Matrix U11T = U11.transpose();
    Matrix ATA = U11T.multiply(U11);

    // -- Step 2: Add λI --
    for (int i = 0; i < n_reg; ++i) {
        ATA(i, i) += lambda;
    }

    // -- Step 3: Compute U11ᵗ * U21 --
    Matrix ATB = U11T.multiply(U21);

    // -- Step 4: Solve (ATA) * X = ATB using dgesv_ --
    double* ATA_data = new double[n_reg * n_reg];
    double* ATB_data = new double[n_reg * k_rhs];

    ATA.toArray(ATA_data, true); // column-major
    ATB.toArray(ATB_data, true);

    int* ipiv_reg = new int[n_reg];
    int info_reg;

    dgesv_(&n_reg, &k_rhs, ATA_data, &n_reg, ipiv_reg, ATB_data, &n_reg, &info_reg);
    if (info_reg != 0) {
        delete[] ATA_data;
        delete[] ATB_data;
        delete[] ipiv_reg;
        throw std::runtime_error("[Tikhonov] Regularized solve failed.");
    }

    // -- Step 5: Wrap result in Matrix --
    Matrix X = Matrix::fromArray(n_reg, k_rhs, ATB_data, true);
    // std::cout << "Xfn: " << frobeniusNorm2(X) << std::endl;

    std::cout << "\nZ: \n"; Z.print();

    // -- Cleanup --
    delete[] ATA_data;
    delete[] ATB_data;
    delete[] ipiv_reg;

    // Compute M = R + Bdᵗ * P * Bd
    Matrix BtPB = B.transpose().multiply(X.multiply(B));
    Matrix M = R.add(BtPB);



    // Dimensions
    int rdim = M.getRows();
    int nrhs = A.getCols(); // K will match columns of Ad_r

    for (int i = 0; i < rdim; ++i) {
        M(i, i) += 1e-2 * M(i, i);  // 0.0001% of diag entry
    }

    std::cout << "BPTBfn: " << frobeniusNorm2(BtPB) << std::endl;

    std::cout << "Mfn: " << frobeniusNorm2(M) << std::endl;

    

    // Form RHS: Bdᵗ * P * Ad
    Matrix BTPA = B.transpose().multiply(X).multiply(A);



    Matrix Kd_r = (M.pseudoInverseJacobi(1e-10, 100)).multiply(BTPA);


    // // Cleanup
    // delete[] M_data;
    // delete[] RHS_data;
    // delete[] ipiv_k;


    // std::cout << "\nX: \n"; X.print();

    delete[] H_data;
    delete[] vs;
    delete[] wr;
    delete[] wi;
    delete[] bwork;
    delete[] work;

    return {Kd_r, X};
}
