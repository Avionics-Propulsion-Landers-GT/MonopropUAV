#include "lqr.h"
#include "Vector.h"
#include "Matrix.h"


LQR lqrController;

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

void LQR::calculateK() {
    Matrix P = Q;                         // Initialize P to Q
    Matrix P_prev(P);          // Previous iteration
    const int num_iterations = 50;

    for (int i = num_iterations; i > 0; --i) {
        P_prev = P;

        Matrix At = A.transpose();
        Matrix Bt = B.transpose();

        Matrix BtP = Bt.multiply(P_prev);
        Matrix AtP = At.multiply(P_prev);

        Matrix term1 = At.multiply(P_prev).multiply(A);
        Matrix term2 = At.multiply(P_prev).multiply(B);
        Matrix term3 = BtP.multiply(B);
        Matrix term4 = R.add(term3).luInverse(); 
        Matrix term5 = term2.multiply(term4).multiply(BtP.multiply(A));

        P = Q.add(term1).add(term5.multiply(-1.0));
    }

    Matrix Bt = B.transpose();
    Matrix BtPB = Bt.multiply(P).multiply(B);
    Matrix inv_term = R.add(BtPB).inverse();
    K = inv_term.multiply(Bt.multiply(P).multiply(A));
}
