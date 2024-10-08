# Sarang Suman
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