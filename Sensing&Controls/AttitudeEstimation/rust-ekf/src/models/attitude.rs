use ndarray::{Array1, Array2, s};
use crate::ekf::model::EKFModel;

pub struct AttitudeModel {
    pub current_time: f64,
    pub previous_time: f64,
    pub delta_time: f64,
}

impl AttitudeModel {
    pub fn new(delta_time: f64) -> Self {
        Self {
            current_time: -delta_time,
            previous_time: -2.0 * delta_time,
            delta_time,
        }
    }
}

impl EKFModel for AttitudeModel {
    fn parse_data(&mut self, data: &[f64]) -> Array1<f64> {
        self.previous_time = self.current_time;
        self.current_time = data[0];
        self.delta_time = self.current_time - self.previous_time;

        Array1::from(vec![
            data[1], data[2], data[3], // gyro
            data[4], data[5], data[6], // accel
            data[7], data[8], data[9], // mag
        ])
    }

    // TODO: Add the state transition function (Dynamics Model)
    fn state_transition_function(&self, state: &Array1<f64>, _dt: f64) -> Array1<f64> {
        state.clone()
    }

    fn state_transition_jacobian(&self, state: &Array1<f64>, _dt: f64) -> Array2<f64> {
        Array2::eye(state.len())
    }

    fn measurement_prediction_function(&self, state: &Array1<f64>) -> Array1<f64> { 
        let rotation_matrix = euler_to_rotation_matrix(state);

        let gyro_pred = Array1::from(vec![0.0, 0.0, 0.0]); 

        let gravity = Array1::from(vec![0.0, 0.0, -9.81]);
        let accel_pred = normalize_vector(&(rotation_matrix.dot(&gravity)));

        let mag_field = Array1::from(vec![0.00005, 0.0, 0.0]);
        let mag_pred = normalize_vector(&(rotation_matrix.dot(&mag_field)));

        let mut result = Array1::zeros(9);
        result.slice_mut(s![0..3]).assign(&gyro_pred);
        result.slice_mut(s![3..6]).assign(&accel_pred);
        result.slice_mut(s![6..9]).assign(&mag_pred);

        result
    }

    fn measurement_prediction_jacobian(&self, state: &Array1<f64>) -> Array2<f64> {
        let n = state.len();
        let mut jac = Array2::zeros((9, n));
        jac[[0, 0]] = 1.0;
        jac[[1, 1]] = 1.0;
        jac[[2, 2]] = 1.0;
        jac
    }
    
    pub fn euler_to_rotation_matrix(euler: &Array1<f64>) -> Array2<f64> {
        let (roll, pitch, yaw) = (euler[0], euler[1], euler[2]);

        let cr = roll.cos();
        let sr = roll.sin();
        let cp = pitch.cos();
        let sp = pitch.sin();
        let cy = yaw.cos();
        let sy = yaw.sin();

        Array2::from_shape_vec(
            (3, 3),
            vec![
                cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
                sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
                -sp, cp * sr, cp * cr,
            ],
        )
        .unwrap()
    }

    pub fn normalize_vector(v: &Array1<f64>) -> Array1<f64> {
        let norm = v.mapv(|x| x * x).sum().sqrt();
        if norm == 0.0 { v.clone() } else { v / norm }
    }
}
