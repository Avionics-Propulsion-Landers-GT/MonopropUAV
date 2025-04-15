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
      B(12, 12, 0.0),
      Q(12, 12, 0.0),
      R(12, 12, 0.0),
      K(12, 12, 0.0),
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


void LQR::calculateK(double dt) {
    // Solve Continuous Algebraic Riccati Equation
    unsigned int n_u = B.getCols();
    double rankEps = 1e-6;
    int maxIter = 100;

    // 2. Build the controllability matrix
    Matrix C = A.controllabilityMatrix(B);

    // 3. Compute thin SVD on the working matrix
    unsigned int m = C.getRows();
    unsigned int n = C.getCols();
    Matrix U(m, n, 0.0);
    Matrix Sigma(n, n, 0.0);
    Matrix V(n, n, 0.0);
    Matrix C_T = C.transpose();
    C_T.thinJacobiSVD(U, Sigma, V, rankEps, maxIter); // Do C_transpose for sanity

    // std::cout << "[LQR] Controllability matrix size: " << C.getRows() << "x" << C.getCols() << "\n";
    // std::cout << "\nC:" << std::endl; C.print();
    std::cout << "B:\n"; B.print();
    std::cout << "A:\n"; A.print();
    // std::cout << "\nU:" << std::endl; U.print(); 
    // std::cout << "\nV:" << std::endl; V.print();
    // std::cout << "\nSigma:" << std::endl;  Sigma.print();

    // std::cout << "Rank of C: " << C.rank(1e-8) << std::endl;
    
    // 4. Determine rank from singular values
    int rank = 0;
    Matrix activations(m, 1, 0);
    for (unsigned int i = 0; i < m; ++i) {
        if (Sigma(i, i) > 1e-8) {
            ++rank;
            activations(i, 0) = 1;
        }
    }

    Matrix T(V.getRows(), A.getRows(), 0.0);  // Final Kalman transformation T (n x n)

    int t_col = 0;
    for (int i = 0; i < activations.getRows(); ++i) {
        if (activations(i, 0) == 1) {  // controllable
            for (int row = 0; row < V.getRows(); ++row) {
                T(row, t_col) = V(row, i);
            }
            ++t_col;
        }
    }

    for (int i = 0; i < activations.getRows(); ++i) {
        if (activations(i, 0) == 0) {  // uncontrollable
            for (int row = 0; row < V.getRows(); ++row) {
                T(row, t_col) = V(row, i);
            }
            ++t_col;
        }
    }

    // 6. Project system into Kalman space
    Matrix Tt = T.transpose(); // Tᵗ
    Matrix A_kal = Tt.multiply(A.multiply(T));     // Tᵗ A T
    Matrix Q_kal = Tt.multiply(Q.multiply(T));     // Tᵗ Q T
    Matrix B_kal = Tt.multiply(B);                 // Tᵗ B

    // 7. Extract controllable subspace
    Matrix A_r(rank, rank, 0.0);
    for (int i = 0; i < rank; ++i) {
        for (int j = 0; j < rank; ++j) {
            A_r(i, j) = A_kal(i, j);
        }
    }

    Matrix Q_r(rank, rank, 0.0);
    for (int i = 0; i < rank; ++i) {
        for (int j = 0; j < rank; ++j) {
            Q_r(i, j) = Q_kal(i, j);
        }
    }

    Matrix B_r(rank, B_kal.getCols(), 0.0);
    for (int i = 0; i < rank; ++i) {
        for (int j = 0; j < B_kal.getCols(); ++j) {
            B_r(i, j) = B_kal(i, j);
        }
    }

    Matrix Tc(T.getRows(), rank, 0.0);  // (n x r)
    for (int row = 0; row < T.getRows(); ++row) {
        for (int col = 0; col < rank; ++col) {
            Tc(row, col) = T(row, col);
        }
    }

    // std::cout << "[LQR] Rank of controllability matrix: " << rank << "\n";
    // std::cout << "activations: " << std::endl; activations.print();
    // std::cout << "[LQR] T (" << V.getRows() <<", " << rank << "):" << std::endl; T.print();
    // std::cout << "T: " << T.getRows() << "x" << T.getCols() << std::endl;
    // std::cout << "B_d: " << B.getRows() << "x" << B.getCols() << std::endl;
    // std::cout << "B_r: " << B_r.getRows() << "x" << B_r.getCols() << std::endl;
    std::cout << "[CARE-IN] A_r:\n"; A_r.print();
    std::cout << "[CARE-IN] B_r:\n"; B_r.print();
    std::cout << "[CARE-IN] Q_r:\n"; Q_r.print();
    // std::cout << "[CARE-IN] R:\n"; R.print();

    // 8. CARE on reduced system
    Matrix P = Q_r;
    try {
        P = solveCARE(A_r, B_r, Q_r, R);
    } catch (const std::exception& e) {
        std::cerr << "Error solving CARE: " << e.what() << "\n";
    }

    // 9.  Do ZOH Discretization Approximation
    Matrix Ad_r = A_r;  // Matrix(A_r.getRows()).add(A_r.multiply(dt));
    Matrix Bd_r = B_r; // B_r.multiply(dt);
    Matrix BtPB = (Bd_r.transpose()).multiply(P.multiply(Bd_r));
    Matrix term1 = ((R.add(BtPB)).pseudoInverseJacobi(1e-12, 100));
    Matrix Kd_r = term1.multiply(Bd_r.transpose()).multiply(P).multiply(Ad_r);

    // 10. Project back out of Kalman space (works because z = Tc^t*x)
    Matrix Kd = Kd_r.multiply(Tc.transpose());


    std::cout << "[LQR] Kd_r (" << Kd_r.getRows() << "x" << Kd_r.getCols() << "):" << std::endl;
    Kd_r.print();
    
    std::cout << "[LQR] Kd (" << Kd.getRows() << "x" << Kd.getCols() << "):" << std::endl;
    Kd.print();

    // Return Kd
    K = Kd;

}
