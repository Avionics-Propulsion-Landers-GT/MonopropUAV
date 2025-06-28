#include "EKF_gyro.h"

EKF_Gyro::EKF_Gyro(const Vector& initial_state, double delta_time, double q_scalar, double r_scalar, double initial_p)
    : ExtendedKalmanFilterGeneral(initial_state, delta_time, q_scalar, r_scalar, initial_p) {}

Vector EKF_Gyro::parseData(const std::vector<double>& data) {
    // not really sure what the raw data will look like, but assuming it contains roll, pitch, and yaw rates
    Vector parsed_data(3, 0.0); 
    parsed_data[0] = data[0]; // Assuming data[0] is the roll rate
    parsed_data[1] = data[1]; // Assuming data[1] is the pitch rate
    parsed_data[2] = data[2]; // Assuming data[2] is the yaw rate
    return parsed_data;
}

Vector EKF_Gyro::stateTransitionFunction(const Vector& parsed_data) {
    Vector new_state(7, 0.0); 
    // Assuming the state vector is [qw, qx, qy, qz, bias_X, bias_Y, bias_Z]
    Quaternion orientation(state[0], state[1], state[2], state[3]);
    // Quaternion corrected_angular_rates(0, parsed_data[0] - state[4], parsed_data[1] - state[5], parsed_data[2] - state[6]);  // [0; wx-bias_X; wy-bias_Y; wz-bias_Z]
    // Quaternion new_orientation = orientation + (orientation * corrected_angular_rates) * (delta_time / 2.0); // Quaternion integration

    Vector omegas(3, 0.0);
    omegas[0] = (parsed_data[0] - state[4]); // Roll rate minus bias_X
    omegas[1] = (parsed_data[1] - state[5]); // Pitch rate minus bias_Y
    omegas[2] = (parsed_data[2] - state[6]); // Yaw rate minus bias_Z
    double big_omega = omegas.magnitude();
    double delta_theta = big_omega * delta_time;

    Quaternion delta_quat(
        cos(delta_theta / 2.0),
        (delta_theta < 1e-6) ? 0.0 : omegas[0] * sin(delta_theta / 2.0) / big_omega,
        (delta_theta < 1e-6) ? 0.0 : omegas[1] * sin(delta_theta / 2.0) / big_omega,
        (delta_theta < 1e-6) ? 0.0 : omegas[2] * sin(delta_theta / 2.0) / big_omega
    );

    // Normalize the quaternion to avoid drift
    Quaternion new_orientation = (delta_quat * orientation).normalize();

    new_state[0] = new_orientation[0]; // qw
    new_state[1] = new_orientation[1]; // qx
    new_state[2] = new_orientation[2]; // qy
    new_state[3] = new_orientation[3]; // qz
    new_state[4] = state[4]; // bias_X remains the same
    new_state[5] = state[5]; // bias_Y remains the same
    new_state[6] = state[6]; // bias_Z remains the same

    return new_state;
}

Matrix EKF_Gyro::stateTransitionJacobian(const Quaternion& orientation, const Vector& angular_rates) {
    // NOTE: does not account for normalization of the quaternion, which is done in the state transition function
    
    Matrix jacobian(7, 7, 0.0);
    double w = orientation[0]; // qw
    double x = orientation[1]; // qx
    double y = orientation[2]; // qy
    double z = orientation[3]; // qz

    // Note that angular rates are assumed to be bias corrected, i.e., they are already in the form of [wx - bias_X, wy - bias_Y, wz - bias_Z]
    double omega_x = angular_rates[0]; // wx
    double omega_y = angular_rates[1]; // wy
    double omega_z = angular_rates[2]; // wz

    double OMEGA = sqrt(omega_x * omega_x + omega_y * omega_y + omega_z * omega_z);

    double half_theta = OMEGA * delta_time / 2.0;
    double s = sin(half_theta);
    double c = cos(half_theta);
    
    // Delta w, x, y, and z
    double delta_w = c;
    double delta_x = (OMEGA < 1e-6) ? 0.0 : omega_x * s / OMEGA;
    double delta_y = (OMEGA < 1e-6) ? 0.0 : omega_y * s / OMEGA;
    double delta_z = (OMEGA < 1e-6) ? 0.0 : omega_z * s / OMEGA;
    // Fill ∂q_new / ∂q
    jacobian(0, 0) =  delta_w;  jacobian(0, 1) = -delta_x;  jacobian(0, 2) = -delta_y;  jacobian(0, 3) = -delta_z;
    jacobian(1, 0) =  delta_x;  jacobian(1, 1) =  delta_w;  jacobian(1, 2) = -delta_z;  jacobian(1, 3) =  delta_y;
    jacobian(2, 0) =  delta_y;  jacobian(2, 1) =  delta_z;  jacobian(2, 2) =  delta_w;  jacobian(2, 3) = -delta_x;
    jacobian(3, 0) =  delta_z;  jacobian(3, 1) = -delta_y;  jacobian(3, 2) =  delta_x;  jacobian(3, 3) =  delta_w;

    // Fill ∂q_w / ∂bias
    double d_delta_qw_d_bx = delta_time * 0.5 * (x * sin(OMEGA * delta_time / 2.0) / OMEGA);
    double d_delta_qw_d_by = delta_time * 0.5 * (y * sin(OMEGA * delta_time / 2.0) / OMEGA);
    double d_delta_qw_d_bz = delta_time * 0.5 * (z * sin(OMEGA * delta_time / 2.0) / OMEGA);

    // Calculate ∂b/∂Δq
    double d_delta_qx_d_bx = calc_delta_q_bias(omega_x, omega_x, OMEGA, 1.0, delta_time);
    double d_delta_qy_d_by = calc_delta_q_bias(omega_y, omega_y, OMEGA, 1.0, delta_time);
    double d_delta_qz_d_bz = calc_delta_q_bias(omega_z, omega_z, OMEGA, 1.0, delta_time);
    double d_delta_qx_d_by = calc_delta_q_bias(omega_x, omega_y, OMEGA, 0.0, delta_time);
    double d_delta_qx_d_bz = calc_delta_q_bias(omega_x, omega_z, OMEGA, 0.0, delta_time);
    double d_delta_qy_d_bx = calc_delta_q_bias(omega_y, omega_x, OMEGA, 0.0, delta_time);
    double d_delta_qy_d_bz = calc_delta_q_bias(omega_y, omega_z, OMEGA, 0.0, delta_time);
    double d_delta_qz_d_bx = calc_delta_q_bias(omega_z, omega_x, OMEGA, 0.0, delta_time);
    double d_delta_qz_d_by = calc_delta_q_bias(omega_z, omega_y, OMEGA, 0.0, delta_time);
    
    // Calculate ∂(Δq ⊗ q)/∂b
    // ∂qw'/ ∂b
    double d_qw_d_bx = w * d_delta_qw_d_bx - x * d_delta_qx_d_bx - y * d_delta_qy_d_bx - z * d_delta_qz_d_bx;
    double d_qw_d_by = w * d_delta_qw_d_by - x * d_delta_qx_d_by - y * d_delta_qy_d_by - z * d_delta_qz_d_by;
    double d_qw_d_bz = w * d_delta_qw_d_bz - x * d_delta_qx_d_bz - y * d_delta_qy_d_bz - z * d_delta_qz_d_bz;
    // ∂qx'/ ∂b
    double d_qx_d_bx = x * d_delta_qw_d_bx + w * d_delta_qx_d_bx + z * d_delta_qy_d_bx - y * d_delta_qz_d_bx;
    double d_qx_d_by = x * d_delta_qw_d_by + w * d_delta_qx_d_by + z * d_delta_qy_d_by - y * d_delta_qz_d_by;
    double d_qx_d_bz = x * d_delta_qw_d_bz + w * d_delta_qx_d_bz + z * d_delta_qy_d_bz - y * d_delta_qz_d_bz;
    // ∂qy'/ ∂b
    double d_qy_d_bx = y * d_delta_qw_d_bx - z * d_delta_qx_d_bx + w * d_delta_qy_d_bx + x * d_delta_qz_d_bx;
    double d_qy_d_by = y * d_delta_qw_d_by - z * d_delta_qx_d_by + w * d_delta_qy_d_by + x * d_delta_qz_d_by;
    double d_qy_d_bz = y * d_delta_qw_d_bz - z * d_delta_qx_d_bz + w * d_delta_qy_d_bz + x * d_delta_qz_d_bz;
    // ∂qz'/ ∂b
    double d_qz_d_bx = z * d_delta_qw_d_bx + y * d_delta_qx_d_bx - x * d_delta_qy_d_bx + w * d_delta_qz_d_bx;
    double d_qz_d_by = z * d_delta_qw_d_by + y * d_delta_qx_d_by - x * d_delta_qy_d_by + w * d_delta_qz_d_by;
    double d_qz_d_bz = z * d_delta_qw_d_bz + y * d_delta_qx_d_bz - x * d_delta_qy_d_bz + w * d_delta_qz_d_bz;
    
    // Fill ∂q_x / ∂bias
    jacobian(0, 4) = d_qw_d_bx;
    jacobian(0, 5) = d_qw_d_by;
    jacobian(0, 6) = d_qw_d_bz;

    jacobian(1, 4) = d_qx_d_bx;
    jacobian(1, 5) = d_qx_d_by;
    jacobian(1, 6) = d_qx_d_bz;

    jacobian(2, 4) = d_qy_d_bx;
    jacobian(2, 5) = d_qy_d_by;
    jacobian(2, 6) = d_qy_d_bz;

    jacobian(3, 4) = d_qz_d_bx;
    jacobian(3, 5) = d_qz_d_by;
    jacobian(3, 6) = d_qz_d_bz;

    // Fill ∂bias / ∂q
    jacobian(4, 4) = 1.0; // bias_X remains the same
    jacobian(5, 5) = 1.0; // bias_Y remains
    jacobian(6, 6) = 1.0; // bias_Z remains the same

    return jacobian;
}

double calc_delta_q_bias(double omega_i, double omega_j, double OMEGA, double kronecker_delta, double delta_time) {
    double s = sin(OMEGA * delta_time / 2.0);
    double c = cos(OMEGA * delta_time / 2.0);

    if (kronecker_delta != 0 && kronecker_delta != 1) {
        // throw error
        throw std::invalid_argument("Invalid Kronecker delta. Must be 0 or 1.");
    }

    if (OMEGA < 1e-6) {
        return -0.5 * delta_time * kronecker_delta;
    }
    return - delta_time * 0.5 * c * (omega_i / OMEGA) * (omega_j / OMEGA) - s * ((OMEGA*OMEGA*kronecker_delta - omega_i*omega_j) / (OMEGA*OMEGA*OMEGA));
}

Vector EKF_Gyro::measurementPredictionFunction() {

    Vector measurement(6, 0.0);
    Quaternion q(state[0], state[1], state[2], state[3]);
    Matrix R = q.toRotationMatrix();

    // Define world gravity and magnetic field vectors
    Vector world_gravity(3, 0.0);
    world_gravity[2] = -9.81; // [0, 0, -9.81] m/s^2

    Vector world_mag(3, 0.0);

    // Need to adjust world magnetic field vector 
    world_mag[0] = 0.4;
    world_mag[1] = 0.0;
    world_mag[2] = 0.6;

    // Rotate into body frame
    Vector accel = R.multiply(world_gravity);
    Vector mag = R.multiply(world_mag);

    // Fill measurement vector
    measurement[0] = accel[0];
    measurement[1] = accel[1];
    measurement[2] = accel[2];
    measurement[3] = mag[0];
    measurement[4] = mag[1];
    measurement[5] = mag[2];
    return measurement;
}

Matrix EKF_Gyro::measurementPredictionJacobian() {
    Matrix jacobian(6, 7, 0.0); // to map from state to observation space
    // The measurement function maps the quaternion part of the state to predicted accelerometer and magnetometer readings.
    // The Jacobian is the derivative of the rotated gravity and magnetic vectors with respect to the quaternion.

    Quaternion q(state[0], state[1], state[2], state[3]);
    Matrix R = q.toRotationMatrix();

    Vector world_gravity(3, 0.0);
    world_gravity[2] = -9.81;

    Vector world_mag(3, 0.0);
    world_mag[0] = 0.4;
    world_mag[1] = 0.0;
    world_mag[2] = 0.6;

    // Helper lambda to compute d(R(q)*v)/dq
    auto dRqdq = [](const Quaternion& q, const Vector& v) -> Matrix {
        // Returns a 3x4 matrix: derivative of R(q)*v w.r.t. [qw, qx, qy, qz]
        double qw = q[0], qx = q[1], qy = q[2], qz = q[3];
        double vx = v[0], vy = v[1], vz = v[2];
        Matrix J(3, 4, 0.0); //sub Jacobian matrix
        // Fill the Jacobian matrix
        J(0, 0) = 2 * (-vz*qy + vy*qz);
        J(0, 1) = 2 * (vy*qy + vz*qz);
        J(0, 2) = 2 * (-2*vx*qy + vy*qx - vz*qw);
        J(0, 3) = 2 * (-2*vx*qz + vy*qw + vz*qx);

        J(1, 0) = 2 * (vz*qx - vx*qz);
        J(1, 1) = 2 * (-2*vy*qx + vx*qy + vz*qw);
        J(1, 2) = 2 * (vx*qx + vz*qz);
        J(1, 3) = 2 * (-vx*qw + vz*qy - 2*vy*qz);

        J(2, 0) = 2 * (-vy*qx + vx*qy);
        J(2, 1) = 2 * (vx*qz - vy*qw - 2*vz*qx);
        J(2, 2) = 2 * (vx*qw + vy*qz - 2*vz*qy);
        J(2, 3) = 2 * (vx*qx + vy*qy);

        return J;
    };

    // Compute Jacobians for gravity and mag
    Matrix Jg = dRqdq(q, world_gravity); // 3x4
    Matrix Jm = dRqdq(q, world_mag);     // 3x4

    // Fill in the Jacobian matrix
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            jacobian(i, j) = Jg(i, j);      // accel rows
            jacobian(i + 3, j) = Jm(i, j);  // mag rows
        }
    }
    // The measurement does not depend on the bias states, so the rest remain zero.
    return jacobian;
}