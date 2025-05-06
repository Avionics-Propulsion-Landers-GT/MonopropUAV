#include "ExtendedKalmanFilterGeneral.h"

ExtendedKalmanFilterGeneral::ExtendedKalmanFilterGeneral(){}

ExtendedKalmanFilterGeneral::ExtendedKalmanFilterGeneral(
    const Vector& initial_state,
    double delta_time,
    double q_scalar,
    double r_scalar,
    double initial_p
)
    : delta_time(delta_time),
      previous_time(0 - 2 * delta_time),
      current_time(0 - delta_time),
      state(initial_state),
      previous_state(initial_state),
      process_noise_covariance(Matrix(initial_state.getRows()).multiply(q_scalar)),
      measurement_noise_covariance(Matrix(initial_state.getRows()).multiply(r_scalar)),
      error_covariance(Matrix(initial_state.getRows()).multiply(initial_p))
{}


void ExtendedKalmanFilterGeneral::predict() {
    Vector predicted_state = stateTransitionFunction();
    Matrix F = stateTransitionJacobian();
    Matrix Ft = F.transpose();
    
    error_covariance = F.multiply(error_covariance).multiply(Ft).add(process_noise_covariance);
}

void ExtendedKalmanFilterGeneral::update(const Vector& measurement, const double dt) {
    delta_time = dt;
    Vector prediction = measurementPredictionFunction();
    Vector residual = measurement.add(Vector(prediction.multiply(-1.0)));  // z - h(x)

    Matrix H = measurementPredictionJacobian();
    Matrix Ht = H.transpose();

    Matrix S = H.multiply(error_covariance).multiply(Ht).add(measurement_noise_covariance);

    Matrix S_inv = S.pseudoInverseJacobi(1e-12, 100); 
    Matrix K = error_covariance.multiply(Ht).multiply(S_inv);  // Kalman gain

    // Update state
    state = state.add(Vector(K.multiply(residual)));

    // Update covariance
    Matrix I = Matrix(error_covariance.getRows()); // Identity
    Matrix KH = K.multiply(H);
    error_covariance = (I.add(KH.multiply(-1.0))).multiply(error_covariance);
}

Vector ExtendedKalmanFilterGeneral::getState() const {
    return state;
}
