# Sarang Suman

"""
Extended Kalman Filter (EKF) implementation for state estimation.

This class provides methods to initialize the EKF, predict the next state, update the state estimate with sensor measurements, and run the filter over a series of sensor readings.

Attributes:
    x (np.ndarray): State vector.
    P (np.ndarray): Error covariance matrix.
    Q (np.ndarray): Process noise covariance matrix.
    R (np.ndarray): Measurement noise covariance matrix.
    state_dim (int): Dimension of the state vector.
    sensor_dim (int): Dimension of the sensor measurements.
    x_symbols (tuple): Symbolic variables for the state vector.


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

Summary:
The EKF iteratively predicts the state of the system and corrects it using new measurements, making it suitable for non-linear systems where the standard Kalman Filter would not be effective.

"""


import numpy as np
import sympy as sp

class EKF():

    # Initializes the matrices used in calculations using the state and sensor dimensions.
    def __init__(self, state_dim, sensor_dim):
        # state_dim: Dimension of the state vector
        # sensor_dim: Dimension of the sensor measurements

        # Initial state estimate (can be provided)
        # Initialize state vector, covariance matrix, process noise covariance, and measurement noise covariance
        self.x = np.zeros((state_dim, 1))
        self.P = np.eye(state_dim)
        self.Q = np.eye(state_dim)
        self.R = np.eye(sensor_dim)

        self.state_dim = state_dim
        self.sensor_dim = sensor_dim

        self.x_symbols = sp.symbols(f'x:{self.state_dim}')  # Create symbolic variables for the state vector

    # Function
    def f(self, x):
        # State transition function (non-linear). Modify as per your system's dynamics
        return x

    def h(self, x):
        # Measurement function (non-linear). Modify as per your sensor's model
        return x[:self.sensor_dim]

    def F_jacobian(self, x):
        # Takes the Jacobian of the state transition function f
        f_sym = sp.Matrix(self.f(sp.Matrix(self.x_symbols)))  # Apply the state transition function to the symbolic state vector
        F = f_sym.jacobian(self.x_symbols)  # Compute the Jacobian matrix of the state transition function
        return np.array(F).astype(np.float64)  # Convert the Jacobian matrix to a NumPy array of type float64

    def H_jacobian(self, x):
        # Takes Jacobian of the measurement function h
        h_sym = sp.Matrix(self.h(sp.Matrix(self.x_symbols)))  # Apply the measurement function to the symbolic state vector
        H = h_sym.jacobian(self.x_symbols)  # Compute the Jacobian matrix of the measurement function
        return np.array(H).astype(np.float64)  # Convert the Jacobian matrix to a NumPy array of type float64

    # Calculates the next state transition, and updates the error covariance
    def predict(self):
        # Prediction step
        F = self.F_jacobian(self.x)
        self.x = self.f(self.x)  # Non-linear state transition
        self.P = F @ self.P @ F.T + self.Q  # Update the error covariance

    # Calculates an update for the state estimate using calculated Kalman Gain and Measurement residual
    # z: sensor measurement
    def update(self, z):
        H = self.H_jacobian(self.x)
        y = z - self.h(self.x)  # Measurement residual (innovation)
        S = H @ self.P @ H.T + self.R  # Innovation covariance
        K = self.P @ H.T @ np.linalg.inv(S)  # Calculate Kalman gain

        self.x = self.x + K @ y  # Updated state estimate
        self.P = (np.eye(self.state_dim) - K @ H) @ self.P  # Updated error covariance

    # Runs iterations of the filter for each sensor reading, calculating new state estimates and updating Kalman gain to optimize the filter
    def run_filter(self, A):
        # A is an n x 3 matrix of sensor data
        self.timestamps = A[:, 0]
        A = np.delete(A, [0], axis=1)

        n = A.shape[0]  # Number of sensor readings

        for i in range(n):
            z = A[i, :].reshape(-1, 1)  # Get the i-th sensor measurement as a column vector
            self.predict()  # Prediction step
            self.update(z)  # Update step

ekf = EKF(3, 9)
a = np.array([[1, 2, 3, 1, 2, 3, 1 ,2 ,3, 4], [4, 5, 6, 1, 2, 3, 1 ,2 ,3, 4], [7, 8, 9,  1, 2, 3, 1 ,2 ,3, 4]])
ekf.run_filter(a)