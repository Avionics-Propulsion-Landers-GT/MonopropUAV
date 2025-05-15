#include "../CustomLinear/Matrix.h"
#include <iostream>
#include <vector>

#ifndef SOLVE_CARE_H
#define SOLVE_CARE_H

std::vector<Matrix> solveCARE(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R);

// Fortran-style SELECT function used for LAPACK's eigenvalue sorting
extern "C" int select_neg_real(double* wr, double* wi);

// LAPACK real Schur decomposition routine with eigenvalue sorting
extern "C" void dgees_(
    char* jobvs, char* sort, int (*select)(double*, double*),
    int* n, double* a, int* lda, int* sdim,
    double* wr, double* wi, double* vs, int* ldvs,
    double* work, int* lwork, bool* bwork, int* info
);

extern "C" void dgesv_(
    int* n, int* nrhs, double* a, int* lda,
    int* ipiv, double* b, int* ldb, int* info
);

#endif // SOLVE_CARE_H