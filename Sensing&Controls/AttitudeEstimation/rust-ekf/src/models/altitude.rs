use ndarray::{Array1, Array2};
use crate::ekf::model::EKFModel;

pub struct AltitudeModel {
    pub current_time: f64,
    pub delta_time: f64,
    pub previous_velocity: f64,
    pub velocity: f64,
}

impl AltitudeModel {
    pub fn new(delta_time: f64) -> Self {
        Self {
            current_time: -delta_time,
            delta_time,
            previous_velocity: 0.0,
            velocity: 0.0,
        }
    }
}

impl EKFModel for AltitudeModel {
    fn parse_data(&mut self, data: &[f64]) -> Array1<f64> {
        self.current_time = data[0];
        Array1::from(vec![data[1]])
    }

    // Todo: Verify with dynamics model
    fn state_transition_function(&self, state: &Array1<f64>, dt: f64) -> Array1<f64> {
        Array1::from(vec![state[0] + self.velocity * dt])
    }

    fn state_transition_jacobian(&self, _state: &Array1<f64>, dt: f64) -> Array2<f64> {
        let acceleration = (self.velocity - self.previous_velocity) / dt;
        let deriv = if self.velocity != 0.0 {
            acceleration / self.velocity
        } else {
            0.0
        };
        Array2::from_shape_vec((1,1), vec![1.0 + deriv]).unwrap()
    }

    fn measurement_prediction_function(&self, state: &Array1<f64>) -> Array1<f64> {
        state.clone()
    }

    fn measurement_prediction_jacobian(&self, _state: &Array1<f64>) -> Array2<f64> {
        Array2::from_shape_vec((1,1), vec![1.0]).unwrap()
    }
}
