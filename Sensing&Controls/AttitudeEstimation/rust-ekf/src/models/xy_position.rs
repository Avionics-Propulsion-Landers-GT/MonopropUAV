use ndarray::{Array1, Array2};
use crate::ekf::model::EKFModel;

pub struct XYPositionModel {
    pub lat0: Option<f64>,
    pub lon0: Option<f64>,
    pub current_time: f64,
    pub delta_time: f64,
}

impl XYPositionModel {
    pub fn new(delta_time: f64) -> Self {
        Self {
            lat0: None,
            lon0: None,
            current_time: -delta_time,
            delta_time,
        }
    }
}

impl EKFModel for XYPositionModel {
    fn parse_data(&mut self, data: &[f64]) -> Array1<f64> {
        if self.lat0.is_none() {
            self.lon0 = Some(data[0]);
            self.lat0 = Some(data[1]);
        }

        // TODO: Double Check the Conversion Formulas
        // Insert Reference to Conversion Formulas here:
        let x = (data[0] - self.lon0.unwrap())
            * self.lat0.unwrap().to_radians().cos()
            * 111320.0;
        let y = (data[1] - self.lat0.unwrap()) * 111139.0;

        self.current_time += self.delta_time;
        Array1::from(vec![x, y])
    }

    fn state_transition_function(&self, state: &Array1<f64>, dt: f64) -> Array1<f64> {
        Array1::from(vec![
            state[0] + state[2] * dt,
            state[1] + state[3] * dt,
            state[2],
            state[3],
        ])
    }

    fn state_transition_jacobian(&self, _state: &Array1<f64>, dt: f64) -> Array2<f64> {
        Array2::from_shape_vec(
            (4, 4),
            vec![
                1.0, 0.0, dt, 0.0,
                0.0, 1.0, 0.0, dt,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ],
        ).unwrap()
    }

    fn measurement_prediction_function(&self, state: &Array1<f64>) -> Array1<f64> {
        Array1::from(vec![state[0], state[1]])
    }

    fn measurement_prediction_jacobian(&self, _state: &Array1<f64>) -> Array2<f64> {
        Array2::from_shape_vec((2, 4), vec![
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
        ]).unwrap()
    }
}
