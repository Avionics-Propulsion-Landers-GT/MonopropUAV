#include "lqr.h"
#include "Vector.h"
#include "Matrix.h"
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

// Solve Riccati Equation
void LQR::calculateK(double dt) {

    unsigned int n_u = B.getCols();
    double rankEps = 1e-6;
    int maxIter = 100;
    // std::cout << "[LQR] Starting solver...\n";

    // 2. Build the controllability matrix
    Matrix C = A.controllabilityMatrix(B);
    // std::cout << "[LQR] Controllability matrix size: " << C.getRows() << "x" << C.getCols() << "\n";


    // Dimensions for SVD
    unsigned int m = C.getRows();
    unsigned int n = C.getCols();
    Matrix U(m, n, 0.0);
    Matrix Sigma(n, n, 0.0);
    Matrix V(n, n, 0.0);

    // Compute thin SVD on the working matrix
    Matrix C_T = C.transpose();
    C_T.thinJacobiSVD(U, Sigma, V, rankEps, maxIter);


    // std::cout << "\nC:" << std::endl; C.print();

    // std::cout << "B_d:\n"; B.print();

    // std::cout << "\nU:" << std::endl; U.print(); 
    // std::cout << "\nV:" << std::endl; V.print();
    // std::cout << "\nSigma:" << std::endl;  Sigma.print();
    

    // 4. Determine rank from singular values
    int rank = 0;
    Matrix activations(m, 1, 0);
    for (unsigned int i = 0; i < m; ++i) {
        if (Sigma(i, i) > rankEps) {
            ++rank;
            activations(i, 0) = 1;
        }
    }
    // std::cout << "[LQR] Rank of controllability matrix: " << rank << "\n";
    // std::cout << "activations: " << std::endl; activations.print();

    // 5. Get basis for controllable subspace: columns of U(:, 0:rank-1)
    Matrix T(V.getRows(), rank, 0.0);  // T will be (n x r)

    int t_col = 0;
    for (int i = 0; i < activations.getRows(); ++i) {
        if (activations(i, 0) == 1) {
            for (int row = 0; row < V.getRows(); ++row) {
                T(row, t_col) = V(row, i);
            }
            ++t_col;
        }
    }

    // std::cout << "[LQR] T (" << V.getRows() <<", " << rank << "):" << std::endl; T.print();

    // 6. Project system onto controllable subspace
    Matrix Tt = T.transpose(); // Tᵗ
    Matrix A_r = Tt.multiply(A.multiply(T));
    Matrix Q_r = Tt.multiply(Q.multiply(T));


    // std::cout << "T: " << T.getRows() << "x" << T.getCols() << std::endl;
    // std::cout << "B_d: " << B.getRows() << "x" << B.getCols() << std::endl;

    Matrix B_r = Tt.multiply(B);

    // std::cout << "B_r: " << B_r.getRows() << "x" << B_r.getCols() << std::endl;


    // std::cout << "[CARE-IN] A_r:\n"; A_r.print();
    // std::cout << "[CARE-IN] B_r:\n"; B_r.print();
    // std::cout << "[CARE-IN] Q_r:\n"; Q_r.print();
    // std::cout << "[CARE-IN] R:\n"; R.print();

    // 7. CARE on reduced system
    Matrix P_r = Q_r;
    Matrix U_b = Matrix(rank, n_u, 0.0);
    Matrix Sigma_b = Matrix(rank, rank, 0.0);

    Matrix V_b = Matrix(rank, rank, 0.0);
    Matrix U_b_t = Matrix(n_u, rank, 0.0);
    Matrix V_b_t = Matrix(rank, rank, 0.0);

    Matrix B_r_t = B_r.transpose();

    B_r_t.thinJacobiSVD(U_b_t, Sigma_b, V_b_t, rankEps, 100);

    Matrix P = Q_r;

    // Solve CARE AtP + PA- PB(R^-1)BtP + Q = 0
    try {
        P = solveCARE(A_r, B_r, Q_r, R);
    
        // P = solveCARE_diagonal(A_r, B_r, Q_r, R);
        // std::cout << "[CARE] Solution matrix P:\n"; P.print();
    } catch (const std::exception& e) {
        // std::cerr << "Error solving CARE: " << e.what() << "\n";
    }



    Matrix Ad_r = Matrix(A_r.getRows()).add(A_r.multiply(dt));
    Matrix Bd_r = B_r.multiply(dt);
    Matrix BtPB = (Bd_r.transpose()).multiply(P.multiply(Bd_r));

    Matrix term1 = ((R.add(BtPB)).pseudoInverse());

    Matrix Kd_eff = term1.multiply(Bd_r.transpose()).multiply(P).multiply(Ad_r);

    // std::cout << "[LQR] Kd_eff (" << Kd_eff.getRows() << "x" << Kd_eff.getCols() << "):" << std::endl;
    // Kd_eff.print();

    // std::cout << "[LQR] Kd_b (" << Kd_b.getRows() << "x" << Kd_b.getCols() << "):" << std::endl;
    // Kd_b.print();

    // Assuming T is orthogonal:




    // std::cout << "[LQR] Projection matrix Tᵗ (" << T.transpose().getRows() << "x" << T.transpose().getCols() << "):\n";
    // T.transpose().print();

    // std::cout << "[LQR] Final gain matrix K (" << K.getRows() << "x" << K.getCols() << "):\n";
    // K.print();


}
