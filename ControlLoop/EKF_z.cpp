#include "EKF_z.h"

/*

    This is the EKF that is used to define the best estimated altitude
    based on either the GPS or Lidar data. GNC people, feel free to tune this 
    appropriately

*/

EKF_Altitude::EKF_Altitude(const Eigen::VectorXd& initial_state, double delta_time, double q_scalar, double r_scalar, double initial_p)
    : ExtendedKalmanFilterGeneral(initial_state, delta_time, q_scalar, r_scalar, initial_p) {}

Eigen::VectorXd EKF_Altitude::parseData(const std::vector<double>& data) {
    Eigen::VectorXd parsed_data(2);  // Adjust size for [z, vz]
    parsed_data.setZero();           // Example initialization
    return parsed_data;
}

Eigen::VectorXd EKF_Altitude::stateTransitionFunction() {
    Eigen::VectorXd new_state(2);
    new_state(0) = state(0) + state(1) * delta_time;  // z' = z + vz * dt
    new_state(1) = state(1);  // vz' = vz (assume constant velocity)
    return new_state;
}

Eigen::MatrixXd EKF_Altitude::stateTransitionJacobian() {
    Eigen::MatrixXd jacobian(2, 2);  
    jacobian << 1, delta_time,  
                0, 1;         
    return jacobian;
}

Eigen::VectorXd EKF_Altitude::measurementPredictionFunction() {
    Eigen::VectorXd measurement(2);
    measurement(0) = state(0);  // Direct altitude measurement
    measurement(1) = state(1); // velocity (init'd to 0)
    return measurement;
}

Eigen::MatrixXd EKF_Altitude::measurementPredictionJacobian() {
    Eigen::MatrixXd jacobian(2, 2);
    jacobian << 1, 0, 
                0, 1;
    return jacobian;
}
