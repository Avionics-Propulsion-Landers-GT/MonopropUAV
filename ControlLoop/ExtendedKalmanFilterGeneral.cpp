#include "ExtendedKalmanFilterGeneral.h"
#include <iostream>

/*

    General EKF class. Modified and based on the work of @Anyi Lin (ExtendedKalmanFilterGeneral.py)

*/

ExtendedKalmanFilterGeneral::ExtendedKalmanFilterGeneral(const Eigen::VectorXd& initial_state, double delta_time, double q_scalar, double r_scalar, double initial_p)
    : delta_time(delta_time), previous_time(0 - 2 * delta_time), current_time(0 - delta_time), state(initial_state) {
    
    process_noise_covariance = Eigen::MatrixXd::Identity(initial_state.size(), initial_state.size()) * q_scalar;
    measurement_noise_covariance = Eigen::MatrixXd::Identity(initial_state.size(), initial_state.size()) * r_scalar;
    error_covariance = Eigen::MatrixXd::Identity(initial_state.size(), initial_state.size()) * initial_p;
}

void ExtendedKalmanFilterGeneral::predict() {
    Eigen::VectorXd predicted_state = stateTransitionFunction();
    Eigen::MatrixXd jacobian = stateTransitionJacobian();
    error_covariance = jacobian * error_covariance * jacobian.transpose() + process_noise_covariance;
}

void ExtendedKalmanFilterGeneral::update(const Eigen::VectorXd& measurement) {
    Eigen::VectorXd measurement_prediction = measurementPredictionFunction();
    Eigen::VectorXd measurement_residual = measurement - measurement_prediction;
    Eigen::MatrixXd jacobian = measurementPredictionJacobian();
    Eigen::MatrixXd residual_covariance = jacobian * error_covariance * jacobian.transpose() + measurement_noise_covariance;
    Eigen::MatrixXd kalman_gain = error_covariance * jacobian.transpose() * residual_covariance.inverse();

    if (residual_covariance.determinant() < 1e-6) {
        std::cerr << "Error: Residual covariance matrix is nearly singular!" << std::endl;
        return;
    }

    // State update
    state += kalman_gain * measurement_residual;

    error_covariance = (Eigen::MatrixXd::Identity(error_covariance.rows(), error_covariance.cols()) - kalman_gain * jacobian) * error_covariance;
}

Eigen::VectorXd ExtendedKalmanFilterGeneral::getState() const {
    return state;
}
