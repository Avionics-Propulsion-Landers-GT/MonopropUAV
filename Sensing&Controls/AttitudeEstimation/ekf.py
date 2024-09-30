# This basic structure is derived and adjusted from ChatGPT's interpretation of an EKF.
# I will rewrite the class and add more specialized code as I learn more about how to implement and EKF

# Sarang Suman

import numpy as np

class EKF():

    # Initializes the matrices used in calculations using the state and sensor dimensions.
    def __init__(self, state_dim, sensor_dim):
        # state_dim: Dimension of the state vector
        # sensor_dim: Dimension of the sensor measurements

        # Initial state estimate (can be provided)
        self.x = np.zeros((state_dim, 1))  # Initial state vector- A vector of Os the size of the state dimension
        self.P = np.eye(state_dim)  # Initial covariance matrix- An identity matrix the size of the covariance matrix
        self.Q = np.eye(state_dim)  # Process noise covariance- An identity matrix the size of the covariance matrix to process its nose
        self.R = np.eye(sensor_dim)  # Measurement noise covariance- An identity matrix the size of the sensor dimension to measure its noise

        self.state_dim = state_dim
        self.sensor_dim = sensor_dim

    # Function
    def f(self, x):
        # State transition function (non-linear). Modify as per your system's dynamics
        return x

    def h(self, x):
        # Measurement function (non-linear). Modify as per your sensor's model
        return x[:self.sensor_dim]

    def F_jacobian(self, x):
        # Takes the Jacobian of the state transition function f
        return np.eye(self.state_dim)


    def H_jacobian(self, x):
        # Takes Jacobian of the measurement function h
        return np.eye(self.sensor_dim, self.state_dim)

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
        n = A.shape[0]  # Number of sensor readings

        for i in range(n):
            z = A[i, :].reshape(-1, 1)  # Get the i-th sensor measurement as a column vector
            self.predict()  # Prediction step
            self.update(z)  # Update step