#include "EKF_z.h"

/*

    This is the EKF that is used to define the best estimated altitude
    based on either the GPS or Lidar data. GNC people, feel free to tune this 
    appropriately.

*/
EKF_Altitude::EKF_Altitude() : ExtendedKalmanFilterGeneral() {}

EKF_Altitude::EKF_Altitude(const Vector& initial_state, double delta_time, double q_scalar, double r_scalar, double initial_p)
    : ExtendedKalmanFilterGeneral(initial_state, delta_time, q_scalar, r_scalar, initial_p) {}

Vector EKF_Altitude::parseData(const std::vector<double>& data) {
    Vector parsed_data(2, 0.0);  // Adjust size for [z, vz]
    // Fill in actual parsing logic if needed
    return parsed_data;
}

Vector EKF_Altitude::stateTransitionFunction() {
    Vector new_state(2, 0.0);

    // z' = z + vz * dt
    new_state(0, 0) = state(0, 0) + state(1, 0) * delta_time;
    new_state(1, 0) = state(1, 0);  // vz' = vz (assume constant velocity)

    return new_state;
}

Matrix EKF_Altitude::stateTransitionJacobian() {
    Matrix jacobian(2, 2, 0.0);
    jacobian(0, 0) = 1.0;
    jacobian(0, 1) = delta_time;
    jacobian(1, 0) = 0.0;
    jacobian(1, 1) = 1.0;

    return jacobian;
}

Vector EKF_Altitude::measurementPredictionFunction() {
    Vector measurement(2, 0.0);
    measurement(0, 0) = state(0, 0);  // z
    measurement(1, 0) = state(1, 0);  // vz
    return measurement;
}

Matrix EKF_Altitude::measurementPredictionJacobian() {
    Matrix jacobian(2, 2, 0.0);
    jacobian(0, 0) = 1.0;
    jacobian(0, 1) = 0.0;
    jacobian(1, 0) = 0.0;
    jacobian(1, 1) = 1.0;

    return jacobian;
}
