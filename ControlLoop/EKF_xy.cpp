#include "EKF_xy.h"

/*

    This is the EKF for the x,y position. It takes data from the GPS
    and the UWB and returns a best estimated state. GNC people, feel free
    to edit/tune this. 

*/

EKF_Position::EKF_Position(const Vector& initial_state, double delta_time, double q_scalar, double r_scalar, double initial_p)
    : ExtendedKalmanFilterGeneral(initial_state, delta_time, q_scalar, r_scalar, initial_p) {}

// Vector EKF_Position::parseData(const std::vector<double>& data) {
//     Vector parsed_data(4, 0.0);  // Adjust the size accordingly
//     // Fill in actual parsing logic if needed
//     return parsed_data;
// }

Vector EKF_Position::stateTransitionFunction() {
    Vector new_state(4, 0.0);

    // Position update: x += vx * dt, y += vy * dt
    new_state(0, 0) = state(0, 0) + state(2, 0) * delta_time;
    new_state(1, 0) = state(1, 0) + state(3, 0) * delta_time;
    new_state(2, 0) = state(2, 0);
    new_state(3, 0) = state(3, 0);

    return new_state;
}

Matrix EKF_Position::stateTransitionJacobian() {
    Matrix jacobian(4, 4, 0.0);
    
    jacobian(0, 0) = 1.0;
    jacobian(0, 2) = delta_time;
    jacobian(1, 1) = 1.0;
    jacobian(1, 3) = delta_time;
    jacobian(2, 2) = 1.0;
    jacobian(3, 3) = 1.0;

    return jacobian;
}

Vector EKF_Position::measurementPredictionFunction() {
    Vector measurement(4, 0.0);

    measurement(0, 0) = state(0, 0);  // x_pos1
    measurement(1, 0) = state(1, 0);  // y_pos1
    measurement(2, 0) = state(0, 0);  // x_pos2
    measurement(3, 0) = state(1, 0);  // y_pos2

    return measurement;
}

Matrix EKF_Position::measurementPredictionJacobian() {
    Matrix jacobian(4, 4, 0.0);

    jacobian(0, 0) = 1.0;  // Sensor 1 x
    jacobian(1, 1) = 1.0;  // Sensor 1 y
    jacobian(2, 0) = 1.0;  // Sensor 2 x (same as sensor 1)
    jacobian(3, 1) = 1.0;  // Sensor 2 y

    return jacobian;
}
