use ndarray::{Array1, Array2};

/// Trait defining the behavior for different EKF implementations
pub trait EKFModel {
    /// Parse raw sensor data into measurement vector
    fn parse_data(&mut self, data: &[f64]) -> Array1<f64>;

    /// Return the model's most recent timestep if it tracks one internally.
    fn delta_time(&self) -> Option<f64> {
        None
    }

    /// State transition function f(x)
    fn state_transition_function(&self, state: &Array1<f64>, dt: f64) -> Array1<f64>;

    /// Jacobian of state transition function
    fn state_transition_jacobian(&self, state: &Array1<f64>, dt: f64) -> Array2<f64>;

    /// Measurement prediction function h(x)
    fn measurement_prediction_function(&self, state: &Array1<f64>) -> Array1<f64>;

    /// Jacobian of measurement prediction function
    fn measurement_prediction_jacobian(&self, state: &Array1<f64>) -> Array2<f64>;
}