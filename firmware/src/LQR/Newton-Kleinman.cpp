#include <iostream>
#include <cmath>
#include "../CustomLinear/Matrix.h"
#include "../CustomLinear/Vector.h"
#include "lqr.h"

LQR lqrcontroller;

// Solve the discrete Lyapunov equation: P = A_cl^T * P * A_cl + F
// using fixed-point iteration.
Matrix solveDiscreteLyapunov(const Matrix &A_cl, const Matrix &F, double tol, int maxIter) {
    Matrix P = F;  // initial guess: P0 = F
    for (int iter = 0; iter < maxIter; ++iter) {
        Matrix AclT = A_cl.transpose();
        Matrix temp  = AclT.multiply(P).multiply(A_cl);
        Matrix P_next = temp.add(F);
        double diffNorm = lqrcontroller.frobeniusNormDiff(P_next,P);
        if (diffNorm < tol)
            return P_next;
        P = P_next;
    }
    std::cout << "Warning: Discrete Lyapunov solver did not converge in " << maxIter << " iterations." << std::endl;
    return P;
}

// Newton–Kleinman algorithm for the discrete-time algebraic Riccati equation:
// P = A^T * P * A - A^T * P * B (B^T * P * B + R)⁻¹ B^T * P * A + Q
// Uses an initial guess P₀ = Q and iterates until the Frobenius norm of the change is below tol.
Matrix newtonKleinmanDARE(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R, double tol = 1e-8, int maxOuter = 100) {
                            
    Matrix P = Q; // Initial guess P₀ = Q

    if (A.getRows() != A.getCols()) {
        throw std::invalid_argument("newtonKleinmanDARE: Matrix A must be square.");
    }
    
    // Check that B has the same number of rows as A.
    // (B's column count can differ because that represents the number of inputs.)
    if (B.getRows() != A.getRows()) {
        throw std::invalid_argument("newtonKleinmanDARE: Matrix B must have the same number of rows as A.");
    }
    
    // Check that Q is square and has the same dimension as A.
    if (Q.getRows() != A.getRows() || Q.getCols() != A.getRows()) {
        throw std::invalid_argument("newtonKleinmanDARE: Matrix Q must be square and match the dimension of A.");
    }
    
    // Check that R is square and that its dimensions match the number of columns of B,
    // which correspond to the number of control inputs.
    if (R.getRows() != B.getCols() || R.getCols() != B.getCols()) {
        throw std::invalid_argument("newtonKleinmanDARE: Matrix R must be square and match the number of inputs (columns of B).");
    }
    
    Matrix P = Q;

    for (int iter = 0; iter < maxOuter; ++iter) {
        // Compute gain: K = (B^T * P * B + R)⁻¹ * (B^T * P * A)
        Matrix Bt = B.transpose();
        Matrix BtP = Bt.multiply(P);
        Matrix BtPB = BtP.multiply(B);
        Matrix denom = BtPB.add(R);
        Matrix denomInv = denom.pseudoInverseJacobi(1e-10, 100);  // Inverse via LU decomposition as provided
        Matrix BtPA = BtP.multiply(A);
        Matrix K = denomInv.multiply(BtPA);

        // Closed-loop matrix: A_cl = A - B*K
        Matrix BK = B.multiply(K);
        Matrix A_cl = A.add(BK.multiply(-1.0));

        // Compute F = Q + K^T * R * K
        Matrix Kt = K.transpose();
        Matrix RK = R.multiply(K);
        Matrix KtRK = Kt.multiply(RK);
        Matrix F = Q.add(KtRK);

        // Solve the discrete Lyapunov equation: P_new = A_cl^T * P_new * A_cl + F
        Matrix P_new = solveDiscreteLyapunov(A_cl, F, tol, 1000);

        // Check convergence via Frobenius norm of the difference.
        double diffNorm = lqrcontroller.frobeniusNormDiff(P_new, P);
        std::cout << "Iteration " << iter << ", diff norm = " << diffNorm << std::endl;
        if (diffNorm < tol) {
            std::cout << "Newton Kleinman converged in " << iter + 1 << " iterations." << std::endl;
            return P_new;
        }
        P = P_new;
    }
    std::cout << "Newton Kleinman did not converge within " << maxOuter << " iterations." << std::endl;
    return P;
}

// Main function to demonstrate the algorithm on a sample system.
// Uncomment, compile alongside matrix, vector and LQR to view sample output.

// int main() {
//     // Example system (discrete-time):
//     // State matrix A (2x2) and input matrix B (2x1)
//     Matrix A(2, 2, 0.0);
//     A(0, 0) = 0.9;  A(0, 1) = 0.3;
//     A(1, 0) = 0.0;  A(1, 1) = 0.95;

//     Matrix B(2, 1, 0.0);
//     B(0, 0) = 0.1;
//     B(1, 0) = 0.2;

//     // Cost matrices: Q (2x2) and R (1x1); assume they are positive definite.
//     Matrix Q(2, 2, 0.0);
//     Q(0, 0) = 1.0;  Q(0, 1) = 0.0;
//     Q(1, 0) = 0.0;  Q(1, 1) = 1.0;

//     Matrix R(1, 1, 0.0);
//     R(0, 0) = 1.0;

//     double tol = 1e-8;  // Convergence tolerance

//     // Solve the DARE using Newton–Kleinman.
//     Matrix P = newtonKleinmanDARE(A, B, Q, R, tol, 100);

//     std::cout << "Solution P:" << std::endl;
//     P.print();

//     return 0;
// }


