#include "lqr.h"
#include "../CustomLinear/Vector.h"
#include "../CustomLinear/Matrix.h"
#include <cmath>
#include <iostream>
#include <iomanip>
#include "solveCARE.h"


LQR lqrController;

double clamp(double x, double lower, double upper) {
    return std::max(lower, std::min(x, upper));
}

double LQR::frobeniusNormDiff(const Matrix& A, const Matrix& B) const {
    double sum = 0.0;
    for (unsigned int i = 0; i < A.getRows(); ++i) {
        for (unsigned int j = 0; j < A.getCols(); ++j) {
            double diff = A(i, j) - B(i, j);
            sum += diff * diff;
        }
    }
    return std::sqrt(sum);
}

double LQR::frobeniusNorm(const Matrix& A) const {
    double sum = 0.0;
    for (unsigned int i = 0; i < A.getRows(); ++i) {
        for (unsigned int j = 0; j < A.getCols(); ++j) {
            double val = A(i, j);
            sum += val * val;
        }
    }
    return std::sqrt(sum);
}

// Constructor
LQR::LQR()
    : state(12, 0.0),
      A(12, 12, 0.0),
      B(12, 3, 0.0),
      Q(12, 12, 0.0),
      R(3, 3, 0.0),
      K(3, 12, 0.0),
      setPoint(12, 0.0)
{}

LQR::~LQR() {}

// Lvalue setters
void LQR::setA(const Matrix& A) {
    this->A = A;
}

void LQR::setB(const Matrix& B) {
    this->B = B;
}

void LQR::setQ(const Matrix& Q) {
    this->Q = Q;
}

void LQR::setR(const Matrix& R) {
    this->R = R;
}

void LQR::setK(const Matrix& K) {
    this->K = K;
}

void LQR::setState(const Vector& state) {
    this->state = state;
}

// Rvalue setters
void LQR::setA(Matrix&& A) {
    this->A = std::move(A);
}

void LQR::setB(Matrix&& B) {
    this->B = std::move(B);
}

void LQR::setQ(Matrix&& Q) {
    this->Q = std::move(Q);
}

void LQR::setR(Matrix&& R) {
    this->R = std::move(R);
}

void LQR::setK(Matrix&& K) {
    this->K = std::move(K);
}

void LQR::setState(Vector&& state) {
    this->state = std::move(state);
}

double LQR::calculateK(double dt, double fnX) {
    unsigned int n_u = B.getCols();
    double rankEps = 1e-15;
    int maxIter = 100;

    Matrix C = A.controllabilityMatrix(B);
    unsigned int m = C.getRows();
    unsigned int n = C.getCols();

    Matrix U(m, n, 0.0), Sigma(n, n, 0.0), V(n, n, 0.0);
    C.transpose().thinJacobiSVD(U, Sigma, V, rankEps, maxIter);

    int rank = 0;
    Matrix activations(m, 1, 0);
    for (unsigned int i = 0; i < m; ++i) {
        if (Sigma(i, i) > 1e-6) {
            ++rank;
            activations(i, 0) = 1;
        }
    }

    Matrix T(V.getRows(), A.getRows(), 0.0);
    int t_col = 0;
    for (int i = 0; i < activations.getRows(); ++i) {
        if (activations(i, 0) == 1) {
            for (int row = 0; row < V.getRows(); ++row) {
                T(row, t_col) = V(row, i);
            }
            ++t_col;
        }
    }
    for (int i = 0; i < activations.getRows(); ++i) {
        if (activations(i, 0) == 0) {
            for (int row = 0; row < V.getRows(); ++row) {
                T(row, t_col) = V(row, i);
            }
            ++t_col;
        }
    }

    Matrix Tt = T.transpose();
    Matrix A_kal = Tt.multiply(A.multiply(T));
    Matrix Q_kal = Tt.multiply(Q.multiply(T));
    Matrix B_kal = Tt.multiply(B);

    Matrix A_r(rank, rank, 0.0), Q_r(rank, rank, 0.0), B_r(rank, B_kal.getCols(), 0.0);
    for (int i = 0; i < rank; ++i)
        for (int j = 0; j < rank; ++j) {
            A_r(i, j) = A_kal(i, j);
            Q_r(i, j) = Q_kal(i, j);
        }
    for (int i = 0; i < rank; ++i)
        for (int j = 0; j < B_kal.getCols(); ++j)
            B_r(i, j) = B_kal(i, j);

    Matrix Tc(T.getRows(), rank, 0.0);
    for (int row = 0; row < T.getRows(); ++row)
        for (int col = 0; col < rank; ++col)
            Tc(row, col) = T(row, col);

    // Attempt solve
    std::vector<Matrix> results;
    try {
        results = solveCARE(A_r, B_r, Q_r, R);
    } catch (const std::exception& e) {
        std::cerr << "[LQR] CARE solve failed: " << e.what() << "\n";
        return fnX; // Do not update anything
    }

    Matrix Kd_r = results[0];
    Matrix newX = results[1];

    // Stability check
    double fro_new = frobeniusNorm(newX);
    double fro_prev = fnX;
    double relDiff = std::abs(fro_new - fro_prev) / std::max(fro_prev, 1e-8);

    // if (fro_prev < 1e-6) {
    //     std::cout << "[LQR] First Riccati update accepted by default (fro_prev = 0)\n";
    // } else {
    //     double relDiff = std::abs(fro_new - fro_prev) / fro_prev;
    
    //     if (relDiff > 0.2) {
    //         std::cerr << "[LQR] REJECTED Riccati update. prev norm: " << fro_prev
    //                   << " | new norm: " << fro_new
    //                   << " | relDiff: " << relDiff << "\n";
    //         return fnX; // Don't update K or X
    //     }
    // }

    // Accept
    Matrix Kd = Kd_r.multiply(Tc.transpose());
    K = Kd;
    // std::cout << "\nNewX: "; newX.print();
    return frobeniusNorm(newX);
}
