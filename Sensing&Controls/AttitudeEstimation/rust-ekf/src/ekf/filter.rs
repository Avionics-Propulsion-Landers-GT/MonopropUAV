use crate::ekf::model::EKFModel;
use ndarray::{Array1, Array2};
use ndarray_linalg::Inverse;

/// Generic Extended Kalman Filter implementation
pub struct ExtendedKalmanFilter<T: EKFModel> {
    pub state: Array1<f64>,
    pub previous_state: Array1<f64>,
    pub error_covariance: Array2<f64>,
    pub process_noise_covariance: Array2<f64>,
    pub measurement_noise_covariance: Array2<f64>,
    pub delta_time: f64,
    pub current_time: f64,
    pub previous_time: f64,
    pub model: T,
}

impl<T: EKFModel> ExtendedKalmanFilter<T> {
    pub fn new(
        initial_state: Array1<f64>,
        initial_measurements: Array1<f64>,
        delta_time: f64,
        q_scalar: f64,
        r_scalar: f64,
        initial_p: f64,
        model: T,
    ) -> Self {
        let state_size = initial_state.len();
        let measurement_size = initial_measurements.len();

        Self {
            previous_state: initial_state.clone(),
            state: initial_state,
            error_covariance: Array2::eye(state_size) * initial_p,
            process_noise_covariance: Array2::eye(state_size) * q_scalar,
            measurement_noise_covariance: Array2::eye(measurement_size) * r_scalar,
            delta_time,
            current_time: -delta_time,
            previous_time: -2.0 * delta_time,
            model,
        }
    }

    pub fn predict(&mut self) {
        self.previous_state = self.state.clone();
        self.state =
            self.model
                .state_transition_function(&self.state, self.delta_time);

        let f_jac = self
            .model
            .state_transition_jacobian(&self.previous_state, self.delta_time);

        self.error_covariance = f_jac.dot(&self.error_covariance).dot(&f_jac.t())
            + &self.process_noise_covariance;
    }

    fn sync_delta_time_from_model(&mut self) {
        if let Some(model_dt) = self.model.delta_time() {
            if model_dt.is_finite() && model_dt > 0.0 {
                self.delta_time = model_dt;
            }
        }
    }

    fn apply_measurement_update(&mut self, measurement: Array1<f64>) {
        let h_pred = self.model.measurement_prediction_function(&self.state);
        let residual = &measurement - &h_pred;

        let h_jac = self.model.measurement_prediction_jacobian(&self.state);

        let s = h_jac.dot(&self.error_covariance).dot(&h_jac.t())
            + &self.measurement_noise_covariance;

        // TODO: Make sure this error doesn't impact the filter's ability to run (Can we give some default value on error instead of returning directly)
        let s_inv = match s.inv() {
            Ok(m) => m,
            Err(_) => {
                self.error_covariance = &self.error_covariance + &(Array2::<f64>::eye(self.state.len()) * 1e-6); // Prevent panic on singular matrix by adding small diagonal before falling back to prediction
                return;
            }
        };

        let k = self.error_covariance.dot(&h_jac.t()).dot(&s_inv);

        self.previous_state = self.state.clone();
        self.state = &self.state + &k.dot(&residual);

        let identity = Array2::eye(self.state.len());
        self.error_covariance = (identity - k.dot(&h_jac)).dot(&self.error_covariance);
    }

    pub fn update(&mut self, data: &[f64]) {
        let measurement = self.model.parse_data(data);
        self.sync_delta_time_from_model();
        self.apply_measurement_update(measurement);
    }

    /// Parse a timestamped measurement, update dt from the model, then run a
    /// full predict/update cycle using that same sample.
    pub fn step(&mut self, data: &[f64]) {
        let measurement = self.model.parse_data(data);
        self.sync_delta_time_from_model();
        self.predict();
        self.apply_measurement_update(measurement);
    }

    pub fn get_state(&self) -> &Array1<f64> {
        &self.state
    }

    pub fn get_covariance(&self) -> &Array2<f64> {
        &self.error_covariance
    }
}