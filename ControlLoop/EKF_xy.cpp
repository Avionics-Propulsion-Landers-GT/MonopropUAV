#include "EKF_xy.h"

EKF_Position::EKF_Position(const Eigen::VectorXd& initial_state, double delta_time, double q_scalar, double r_scalar, double initial_p)
    : ExtendedKalmanFilterGeneral(initial_state, delta_time, q_scalar, r_scalar, initial_p) {}

Eigen::VectorXd EKF_Position::parseData(const std::vector<double>& data) {
    Eigen::VectorXd parsed_data(4);  // Adjust the size accordingly
    parsed_data.setZero();           // Example initialization
    return parsed_data;
}

Eigen::VectorXd EKF_Position::stateTransitionFunction() {
    Eigen::VectorXd new_state(4);
    new_state(0) = state(0) + state(2) * delta_time;
    new_state(1) = state(1) + state(3) * delta_time;
    new_state(2) = state(2);
    new_state(3) = state(3);
    return new_state;
}

Eigen::MatrixXd EKF_Position::stateTransitionJacobian() {
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(4, 4);
    jacobian(0, 2) = delta_time;
    jacobian(1, 3) = delta_time;
    return jacobian;
}

Eigen::VectorXd EKF_Position::measurementPredictionFunction() {
    Eigen::VectorXd measurement(4);
    measurement(0) = state(0);  // x_pos1 (sensor 1 measurement)
    measurement(1) = state(1);  // y_pos1 (sensor 1 measurement)
    measurement(2) = state(0);  // x_pos2 (sensor 2 measurement)
    measurement(3) = state(1);  // y_pos2 (sensor 2 measurement)
    return measurement;
}

Eigen::MatrixXd EKF_Position::measurementPredictionJacobian() {
    Eigen::MatrixXd jacobian(4, 4);
    
    // The first two rows correspond to Sensor 1's x, y
    // The last two rows correspond to Sensor 2's x, y
    jacobian << 1, 0, 0, 0,  // Sensor 1 measures x
                0, 1, 0, 0,  // Sensor 1 measures y
                1, 0, 0, 0,  // Sensor 2 measures x
                0, 1, 0, 0;  // Sensor 2 measures y
    
    return jacobian;
}
