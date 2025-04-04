#include "lqr.h"
#include "Vector.h"
#include "Matrix.h"
#include <cmath>
#include <iostream>


LQR lqrController;

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
    std::cout<<"A"<<std::endl;
    A.print();
    std::cout<<"B"<<std::endl;
    B.print();
    std::cout<<"Q"<<std::endl;
    Q.print();
    std::cout<<"R"<<std::endl;
    R.print();

    // Discretize A and B
    Matrix A_d = Matrix(A.getRows()).add(A.multiply(dt));
    Matrix B_d = B.multiply(dt);

    // R.pseudoInverse().print();

    Matrix P = A.transpose().multiply(Q).multiply(A);;
    Matrix P_prev = P;
    const double tolerance = 1e-6;
    const int max_iterations = 100;
    int iter = 0;

    for (; iter < max_iterations; ++iter) {
        P_prev = P;

        Matrix BtP = B_d.transpose().multiply(P);
        Matrix BtPB = BtP.multiply(B_d);
        Matrix inv_term = (R.add(BtPB)).luInverse();  // You can switch to pseudoInverse if needed

        Matrix term1 = A_d.transpose().multiply(P).multiply(A_d);
        Matrix term2 = A_d.transpose().multiply(P).multiply(B_d).multiply(inv_term).multiply(BtP).multiply(A_d);

        P = Q.add(term1.add(term2.multiply(-1)));

        // Convergence check
        double diff = frobeniusNormDiff(P, P_prev);
        std::cout << "[DEBUG] Riccati iteration " << iter << " diff: " << diff << std::endl;

        if (frobeniusNormDiff(P, P_prev) < tolerance) break;
    }

    if (iter == max_iterations) {
        std::cerr << "[WARN] DARE did not converge in " << max_iterations << " iterations.\n";
    }

    Matrix K_temp = (R.add(B_d.transpose().multiply(P).multiply(B_d))).luInverse();
    K = K_temp.multiply(B_d.transpose()).multiply(P).multiply(A_d);
}

