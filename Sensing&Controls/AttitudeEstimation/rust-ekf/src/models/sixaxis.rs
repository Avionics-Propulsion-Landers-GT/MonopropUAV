use ndarray::{Array1, Array2, s};
use crate::ekf::model::EKFModel;

/// 6-axis IMU EKF model
/// - Estimates orientation (roll, pitch, yaw) and gyroscope biases
/// - Uses 3-axis gyroscope + 3-axis accelerometer (no magnetometer)
pub struct Imu6AxisModel {
    pub current_time: f64,  // timestamp of latest measurement
    pub previous_time: f64, // timestamp of previous measurement
    pub delta_time: f64,    // difference between current and previous timestamp

    // Store latest gyro reading for predicting next orientation
    pub latest_gyro: Array1<f64>, // [gx, gy, gz]
}

impl Imu6AxisModel {
    /// Create a new IMU model
    pub fn new(delta_time: f64) -> Self {
        Self {
            current_time: -delta_time,        // negative to indicate "no measurements yet"
            previous_time: -2.0 * delta_time, // ensures first dt is valid
            delta_time,
            latest_gyro: Array1::zeros(3),   // start with zero gyro readings
        }
    }
}

impl EKFModel for Imu6AxisModel {

    /// Input format: [timestamp, gx, gy, gz, ax, ay, az]
    /// Returns a 6-element vector: [gx, gy, gz, ax, ay, az]
    fn parse_data(&mut self, data: &[f64]) -> Array1<f64> {
        self.previous_time = self.current_time;
        self.current_time = data[0];
        self.delta_time = self.current_time - self.previous_time;

        // Update the latest gyro measurement (used in state propagation)
        self.latest_gyro = Array1::from(vec![data[1], data[2], data[3]]);

        // Return combined gyro + accel vector for EKF measurement
        Array1::from(vec![
            data[1], data[2], data[3], // gyro
            data[4], data[5], data[6], // accel
        ])
    }

    /// Predicts the next state given the current state and dt
    /// State vector: [roll, pitch, yaw, bgx, bgy, bgz]
    fn state_transition_function(&self, state: &Array1<f64>, dt: f64) -> Array1<f64> {
        let mut new_state = state.clone();

        // Remove gyro biases to get true angular velocities
        let wx = self.latest_gyro[0] - state[3]; // X-axis rotation
        let wy = self.latest_gyro[1] - state[4]; // Y-axis rotation
        let wz = self.latest_gyro[2] - state[5]; // Z-axis rotation

        // Simple Euler integration: new angle = old angle + angular velocity * dt
        new_state[0] += wx * dt; // roll
        new_state[1] += wy * dt; // pitch
        new_state[2] += wz * dt; // yaw

        // Biases are assumed constant (handled by process noise Q in EKF)
        new_state
    }

    /// Linear approximation of state transition function
    /// F shows how each state variable affects the next state
    fn state_transition_jacobian(&self, _state: &Array1<f64>, dt: f64) -> Array2<f64> {
        let mut F = Array2::eye(6); // start with identity (most state unchanged)

        // Roll depends negatively on gyro bias X
        F[[0,3]] = -dt;
        // Pitch depends negatively on gyro bias Y
        F[[1,4]] = -dt;
        // Yaw depends negatively on gyro bias Z
        F[[2,5]] = -dt;

        F
    }

    /// Predict what the IMU would measure if the system were in `state`
    /// - Gyro measures bias (main contributor after removing orientation effect)
    /// - Accelerometer measures rotated gravity vector
    fn measurement_prediction_function(&self, state: &Array1<f64>) -> Array1<f64> {

        // Convert Euler angles to rotation matrix: inertial -> body frame
        let rotation_matrix = Self::euler_to_rotation_matrix(state);

        // Gravity vector in inertial frame (points down)
        let gravity = Array1::from(vec![0.0, 0.0, -9.81]);

        // Predicted accelerometer measurement: rotate gravity into body frame
        let accel_pred = rotation_matrix.dot(&gravity);

        let mut result = Array1::zeros(6);

        // Predicted gyro measurement = current bias
        result[0] = state[3]; // gyro X
        result[1] = state[4]; // gyro Y
        result[2] = state[5]; // gyro Z

        // Predicted accelerometer measurement
        result.slice_mut(s![3..6]).assign(&accel_pred);

        result
    }

    /// Computes partial derivatives of measurement prediction w.r.t. state variables
    /// Uses finite differences: slightly perturb each state variable
    fn measurement_prediction_jacobian(&self, state: &Array1<f64>) -> Array2<f64> {
        let eps = 1e-6;      // small change for numerical derivative
        let n = state.len();  // number of state variables (6)
        let m = 6;            // number of measurements (gyro + accel)

        let mut H = Array2::zeros((m, n));
        let h0 = self.measurement_prediction_function(state);

        for i in 0..n {
            let mut perturbed = state.clone();
            perturbed[i] += eps;          // slightly perturb state variable i

            let hi = self.measurement_prediction_function(&perturbed);
            let diff = (&hi - &h0) / eps; // approximate derivative w.r.t. variable i

            for j in 0..m {
                H[[j, i]] = diff[j];
            }
        }

        H
    }
}

impl Imu6AxisModel {

    /// Convert Euler angles (roll, pitch, yaw) to 3x3 rotation matrix
    /// Rotation matrix rotates vectors from inertial frame -> body frame
    fn euler_to_rotation_matrix(euler: &Array1<f64>) -> Array2<f64> {
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
                -sp,     cp * sr,                cp * cr,
            ],
        )
        .unwrap()
    }
}