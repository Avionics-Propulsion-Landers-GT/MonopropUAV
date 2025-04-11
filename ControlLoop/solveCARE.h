#include "Matrix.h"
#include <iostream>

#ifndef SOLVE_CARE_H
#define SOLVE_CARE_H

#include "Matrix.h"


// Solves the Continuous-time Algebraic Riccati Equation (CARE):
// AᵗX + XA - XBR⁻¹BᵗX + Q = 0
//
// Returns the symmetric positive semi-definite solution X

Matrix solveCARE(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R);

// Fortran-style SELECT function used for LAPACK's eigenvalue sorting
extern "C" int select_neg_real(double* wr, double* wi);

// LAPACK real Schur decomposition routine with eigenvalue sorting
extern "C" void dgees_(
    char* jobvs, char* sort, int (*select)(double*, double*),
    int* n, double* a, int* lda, int* sdim,
    double* wr, double* wi, double* vs, int* ldvs,
    double* work, int* lwork, bool* bwork, int* info
);

extern "C" void dgetrf_(int* m, int* n, double* a, int* lda, int* ipiv, int* info);
extern "C" void dgetri_(int* n, double* a, int* lda, int* ipiv, double* work, int* lwork, int* info);

#endif // SOLVE_CARE_H