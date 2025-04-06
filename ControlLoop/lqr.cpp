#include "lqr.h"
#include "Vector.h"
#include "Matrix.h"
#include <cmath>
#include <iostream>
#include <iomanip>


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
    double rankEps = 1e-1;
    int maxIter = 100;
    std::cout << "[LQR] Starting solver...\n";

    // 2. Build the controllability matrix
    Matrix C = A.controllabilityMatrix(B);
    std::cout << "[LQR] Controllability matrix size: " << C.getRows() << "x" << C.getCols() << "\n";


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
    std::cout << "[LQR] Rank of controllability matrix: " << rank << "\n";
    std::cout << "activations: " << std::endl; activations.print();

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
    Matrix R_r = Tt.multiply(R.multiply(T));


    std::cout << "Tᵗ: " << Tt.getRows() << "x" << Tt.getCols() << std::endl;
    std::cout << "B_d: " << B.getRows() << "x" << B.getCols() << std::endl;

    Matrix B_r = Tt.multiply(B);

    std::cout << "B_r: " << B_r.getRows() << "x" << B_r.getCols() << std::endl;


    // std::cout << "[LQR] A_r:\n"; A_r.print();
    // std::cout << "[LQR] B_r:\n"; B_r.print();
    // std::cout << "[LQR] Q_r:\n"; Q_r.print();

    // 7. DARE on reduced system
    Matrix P_r = Q_r;
    Matrix U_b = Matrix(rank, n_u, 0.0);
    Matrix Sigma_b = Matrix(rank, rank, 0.0);

    Matrix V_b = Matrix(rank, rank, 0.0);
    Matrix U_b_t = Matrix(n_u, rank, 0.0);
    Matrix V_b_t = Matrix(rank, rank, 0.0);

    Matrix B_r_t = B_r.transpose();

    // Step 1: SVD on B_r_t (n_u x rank)
    B_r_t.thinJacobiSVD(U_b_t, Sigma_b, V_b_t, rankEps, 100);


    U_b = V_b_t.transpose();
    V_b = U_b_t;

    int rank_b = 0;
    Matrix activations_b = Matrix(rank, 1, 0);
    for (unsigned int i = 0; i < Sigma_b.getRows(); ++i) {
        if (Sigma_b(i, i) > rankEps) {
            ++rank_b;
            activations_b(i,0) = 1;
        }
    }
    std::cout << "[LQR] Effective input rank: " << rank_b << "\n";
    std::cout << "sactivations_b " << std::endl; activations_b.print();

    Matrix U_eff = Matrix(U_b.getRows(), rank_b, 0);

    int u_col = 0;
    for (int i = 0; i < activations_b.getRows(); ++i) {
        if (activations_b(i, 0) == 1) {
            for (int row = 0; row < U_b.getRows(); ++row) {
                U_eff(row, u_col) = U_b(row, i);
            }
            ++u_col;
        }
    }

    Matrix Sigma_eff(rank_b, rank_b, 0.0);
    int sigma_col = 0;
    for (int i = 0; i < Sigma_b.getRows(); ++i) {
        if (activations_b(i, 0) == 1) {
            Sigma_eff(sigma_col, sigma_col) = Sigma_b(i, i);
            ++sigma_col;
        }
    }

    

    Matrix V_b_eff(V_b.getRows(), rank_b, 0.0); // (n_u x rank_b)
    int v_col = 0;
    for (int i = 0; i < activations_b.getRows(); ++i) {
        if (activations_b(i, 0) == 1) {
            for (int row = 0; row < V_b.getRows(); ++row) {
                V_b_eff(row, v_col) = V_b(row, i);
            }
            ++v_col;
        }
    }

    Matrix B_eff = Sigma_eff;



    Matrix A_eff = U_eff.transpose().multiply(A_r.multiply(U_eff)); // A_eff = U_bᵗ * A_r * U_b
    Matrix Q_eff = U_eff.transpose().multiply(Q_r.multiply(U_eff)); // Q_eff = U_bᵗ * Q_r * U_b
    Matrix P_eff = U_eff.transpose().multiply(P_r.multiply(U_eff));
    Matrix R_eff = V_b_eff.transpose().multiply(R.multiply(V_b_eff));

    Matrix A_r_d = Matrix(rank_b).add(A_eff);
    Matrix B_r_d = B_eff;

    std::cout << "[LQR] Sigma_b (" << Sigma_b.getRows() << "x" << Sigma_b.getCols() << "):" << std::endl;
    Sigma_b.print();

    std::cout << "[LQR] U_b (" << U_b.getRows() << "x" << U_b.getCols() << "):" << std::endl;
    U_b.print();

    std::cout << "[LQR] V_b (" << V_b.getRows() << "x" << V_b.getCols() << "):" << std::endl;
    V_b.print();

    std::cout << "[LQR] Sigma_eff (" << Sigma_eff.getRows() << "x" << Sigma_eff.getCols() << "):" << std::endl;
    Sigma_eff.print();

    std::cout << "[LQR] U_eff (" << U_eff.getRows() << "x" << U_eff.getCols() << "):" << std::endl;
    U_eff.print();

    std::cout << "[LQR] V_eff (" << V_b_eff.getRows() << "x" << V_b_eff.getCols() << "):" << std::endl;
    V_b_eff.print();

    std::cout << "[LQR] B_eff (" << B_eff.getRows() << "x" << B_eff.getCols() << "):" << std::endl;
    B_eff.print();

    std::cout << "[LQR] Q_eff (" << Q_eff.getRows() << "x" << Q_eff.getCols() << "):" << std::endl;
    Q_eff.print();

    std::cout << "[LQR] A_eff (" << A_eff.getRows() << "x" << A_eff.getCols() << "):" << std::endl;
    A_eff.print();

    std::cout << "[LQR] R_eff (" << R_eff.getRows() << "x" << R_eff.getCols() << "):" << std::endl;
    R_eff.print();

    std::cout << "[LQR] P_eff (" << P_eff.getRows() << "x" << P_eff.getCols() << "):" << std::endl;
    P_eff.print();

    // Matrix test = A_r.multiply(U_eff);  std::cout << "\nTest:\n "; test.print();

    Matrix K_eff = Matrix(rank_b, rank_b, 0);

    Matrix P_prev(rank_b, rank_b, 0);

    Matrix P_next = P_prev;

    P_prev = P_eff;

    int iter = 0;

    double alpha = 0.05;

    for (; iter < maxIter; ++iter) {
        // CARE Iteration step
        Matrix gain = (R_eff.pseudoInverse())
                        .multiply(B_eff.transpose())
                        .multiply(P_eff);
        Matrix term2 = P_eff.multiply(B_eff).multiply(gain);
        Matrix P_next = Q_eff.add(term2.multiply(-1));  // Updated P_next from current P_eff
    
        // Compute difference between iterations for convergence check
        Matrix deltaP = P_next.add(P_eff.multiply(-1));         // Proper deltaP
        double diff = frobeniusNorm(deltaP);            // Frobenius norm of the delta
        double norm_deltaP = frobeniusNorm(deltaP);
        double norm_P = frobeniusNorm(P_eff);
        double ratio = norm_P / (norm_deltaP + 1e-8);   // Stability indicator, optional
    
        std::cout << "[CARE] Iter " << iter << "  ΔP = " << diff << "\n";
    
        if (diff < rankEps) break;
    
        // Update P_eff for the next iteration
        P_eff = P_next;
    }

    if (iter == maxIter) {
        std::cerr << "[LQR] Warning: Riccati solver did not converge.\n";
    } else {
        std::cout << "[LQR] CARE converged in " << iter << " iterations.\n";
    
    }

    std::cout << "[LQR] P_eff (" << P_eff.getRows() << "x" << P_eff.getCols() << "):" << std::endl; P_eff.print();


    // 8. Compute gain on reduced system
    Matrix B_eff_dt = B_eff.multiply(dt);
    Matrix A_eff_dt = Matrix(rank_b).add(A_eff.multiply(dt));
                           
    Matrix Bt_P = B_eff_dt.transpose().multiply(P_eff);        // (1x1)
    Matrix BtPB = Bt_P.multiply(B_eff_dt);                     // (1x1)
    Matrix toInvert = R_eff.add(BtPB);                         // (1x1)

    Matrix inv = toInvert.pseudoInverse(); // (1x1) inverse
    K_eff = inv.multiply(Bt_P).multiply(A_eff_dt);      // (1x1)

    // Print for verification
    std::cout << "[LQR] Continuous gain K_eff (" << K_eff.getRows() << "x" << K_eff.getCols() << "):\n";
    (K_eff.multiply(1000)).print();
    
    
    //9. Project gain back to full system: K = K_r * Tᵗ
    Matrix K_r = K_eff.multiply(U_b.transpose()); // From U_b basis back to original reduced space
    Matrix K = K_r.multiply(T.transpose()); // From reduced to full state space

    std::cout << "[LQR] Reduced gain K_r (" << K_r.getRows() << "x" << K_r.getCols() << "):\n";
    K_r.print();

    std::cout << "[LQR] Projection matrix Tᵗ (" << T.transpose().getRows() << "x" << T.transpose().getCols() << "):\n";
    T.transpose().print();

    std::cout << "[LQR] Final gain matrix K (" << K.getRows() << "x" << K.getCols() << "):\n";
    K.print();


}
