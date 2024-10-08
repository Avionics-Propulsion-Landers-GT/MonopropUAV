# Extended Kalman Filter (EKF) 
## Implementation for state estimation.

This class provides methods to initialize the EKF, predict the next state, update the state estimate with sensor measurements, and run the filter over a series of sensor readings.

**Attributes:**
*     x (np.ndarray): State vector.
*     P (np.ndarray): Error covariance matrix.
*     Q (np.ndarray): Process noise covariance matrix.
*     R (np.ndarray): Measurement noise covariance matrix.
*     state_dim (int): Dimension of the state vector.
*     sensor_dim (int): Dimension of the sensor measurements.
*     x_symbols (tuple): Symbolic variables for the state vector.


An Extended Kalman Filter (EKF) is an algorithm used for estimating the state of a dynamic system from a series of incomplete and noisy measurements. It extends the basic Kalman Filter to handle non-linear systems. Here is a brief overview of its main components and steps:

1. State Vector (`x`): Represents the estimated state of the system.
2. Error Covariance Matrix (`P`): Represents the uncertainty in the state estimate.
3. Process Noise Covariance Matrix (`Q`): Represents the uncertainty in the process model.
4. Measurement Noise Covariance Matrix (`R`): Represents the uncertainty in the measurements.

Main Steps:

1. Prediction Step:
   - State Transition Function (`f(x)`): Predicts the next state based on the current state.
   - Jacobian of State Transition Function (`F_jacobian(x)`): Linearizes the state transition function around the current state.
   - Update State Estimate: Uses the state transition function to predict the next state.
   - Update Error Covariance: Uses the Jacobian to update the error covariance matrix.

2. Update Step:
   - Measurement Function (`h(x)`): Maps the predicted state to the measurement space.
   - Jacobian of Measurement Function (`H_jacobian(x)`): Linearizes the measurement function around the predicted state.
   - Measurement Residual (`y`): Difference between the actual measurement and the predicted measurement.
   - Innovation Covariance (`S`): Combines the error covariance and measurement noise.
   - Kalman Gain (`K`): Determines how much the predictions should be corrected based on the measurement residual.
   - Update State Estimate: Corrects the predicted state using the Kalman Gain and measurement residual.
   - Update Error Covariance: Updates the error covariance matrix to reflect the correction.

**Summary:**

The EKF iteratively predicts the state of the system and corrects it using new measurements, making it suitable for non-linear systems where the standard Kalman Filter would not be effective.
