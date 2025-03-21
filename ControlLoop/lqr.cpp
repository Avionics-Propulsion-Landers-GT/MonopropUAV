#include "lqr.h"

// Constructor
LQR::LQR () {
    state = Eigen::VectorXd::Zero(12);
    A = Eigen::MatrixXd::Zero(12, 12);
    B = Eigen::MatrixXd::Zero(12, 12);
    Q = Eigen::MatrixXd::Zero(12, 12);
    R = Eigen::MatrixXd::Zero(12, 12);
    K = Eigen::MatrixXd::Zero(12, 12);
    setPoint = Eigen::VectorXd::Zero(12);
}

LQR::~LQR() {}

//lvalue setters

void LQR::setA(const Eigen::MatrixXd& A) {
    this->A = A;
}

void LQR::setB(const Eigen::MatrixXd& B) {
    this->B = B;
}

void LQR::setQ(const Eigen::MatrixXd& Q) {
    this->Q = Q;
}

void LQR::setR(const Eigen::MatrixXd& R) {
    this->R = R;
}

void LQR::setK(const Eigen::MatrixXd& K) {
    this->K = K;
}

void LQR::setState(const Eigen::VectorXd& state) {
    this->state = state;
}

//rvalue setters (for processing efficiency if system is time variant and our matrices keep changing)

void LQR::setA(Eigen::MatrixXd&& A) {
    this->A = std::move(A);
}

void LQR::setB(Eigen::MatrixXd&& B) {
    this->B = std::move(B);
}

void LQR::setQ(Eigen::MatrixXd&& Q) {
    this->Q = std::move(Q);
}

void LQR::setR(Eigen::MatrixXd&& R) {
    this->R = std::move(R);
}

void LQR::setK(Eigen::MatrixXd&& K) {
    this->K = std::move(K);
}

void LQR::setState(Eigen::VectorXd&& state) {
    this->state = std::move(state);
}

void LQR::calculateK() {
    Eigen::MatrixXd P = Q; // Initialize P to Q
    Eigen::MatrixXd P_prev; // To store the previous P    
    const int num_iterations = 50;
    
    //finite horizon LQR
    for (int i = num_iterations; i > 0; --i) {
        P_prev = P;
        P = Q + A.transpose() * P_prev * A - (A.transpose() * P_prev * B) * ((R + B.transpose() * P_prev * B).inverse()) * (B.transpose() * P_prev * A);
    }

    // Calculate K
    K = (R+B.transpose() * P * B).inverse() * (B.transpose() * P * A);
}


