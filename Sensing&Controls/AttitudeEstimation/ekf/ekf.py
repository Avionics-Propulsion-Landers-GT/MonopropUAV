import numpy as np
import sympy as sp
import pandas as pd
import copy
import csv
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt

df = pd.read_csv('../TestBenches/noisy_monocopter_data.csv')
df = df.iloc[:, [1, 2, 3]]
# df = df.drop(index = 0)
arr = df.to_numpy()

validation_df = pd.read_csv('../eulers_data.csv')
validation_df = validation_df.iloc[: , 1:] # Remove the first column
validation_data = validation_df.to_numpy()



class EKF():

    # Initializes the matrices used in calculations using the state and sensor dimensions.
    def __init__(self, state_dim, sensor_dim):
        # state_dim: Dimension of the state vector
        # sensor_dim: Dimension of the sensor measurements

        # Initial state estimate (can be provided)
        # Initialize state vector, covariance matrix, process noise covariance, and measurement noise covariance
        self.x = np.zeros((state_dim, 1))
        self.P = np.eye(state_dim)
        self.Q = np.eye(state_dim) * .1
        self.R = (np.eye(sensor_dim)) * 225
        self.prevX = np.zeros((state_dim, 1))

        self.state_dim = state_dim
        self.sensor_dim = sensor_dim

        self.x_symbols = sp.symbols(f'x:{self.state_dim}')  # Create symbolic variables for the state vector

    # Function
    def f(self, x):
        # State transition function (non-linear). Modify as per your system's dynamics
        return x + (x - self.prevX)

    def h(self, x):
        # Measurement function (non-linear). Modify as per your sensor's model
        #print(x[:self.sensor_dim])
        return x[:self.sensor_dim]

    def F_jacobian(self, x):
        '''# Takes the Jacobian of the state transition function f
        f_sym = sp.Matrix(self.f(sp.Matrix(self.x_symbols)))  # Apply the state transition function to the symbolic state vector
        F = f_sym.jacobian(self.x_symbols)  # Compute the Jacobian matrix of the state transition function
        return np.array(F).astype(np.float64)  # Convert the Jacobian matrix to a NumPy array of type float64
        '''
        return ((self.x - self.prevX)).T
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
        return(self.x) # Return the predicted state estimate

    # Calculates an update for the state estimate using calculated Kalman Gain and Measurement residual
    # z: sensor measurement
    def update(self, z):
        H = self.H_jacobian(self.x)
        y = z - self.h(self.x)  # Measurement residual (innovation)
        S = H @ self.P @ H.T + self.R  # Innovation covariance
        K = self.P @ H.T @ np.linalg.inv(S)  # Calculate Kalman gain
        
        self.prevX = copy.deepcopy(self.x)
        self.x = self.x + K @ y  # Updated state estimate
        self.P = (np.eye(self.state_dim) - K @ H) @ self.P  # Updated error covariance

    # Runs iterations of the filter for each sensor reading, calculating new state estimates and updating Kalman gain to optimize the filter
    def run_filter(self, A):
        measurements = []
        n = A.shape[0]

        for i in range(n):
            z = A[i, :].reshape(-1, 1)  # Get the i-th sensor measurement as a column vector

            measurement = [float(''.join(map(str, coord))) for coord in self.predict().tolist()]
            measurements.append(measurement)

            self.update(z)  # Update step

        with open('predictions.csv', 'w', newline='') as file:
            # Step 4: Using csv.writer to write the list to the CSV file
            writer = csv.writer(file)
            writer.writerows(measurements)

        return measurements

ekf = EKF(3,3)
ekf.run_filter(arr)

measurements = pd.read_csv('predictions.csv', header=None)


# Plot x values
plt.figure()
plt.plot(measurements[measurements.columns[0]], label='Predicted x')
plt.plot(validation_df[validation_df.columns[0]], label='Actual x')
plt.xlabel('Time step')
plt.ylabel('x value')
plt.legend()
plt.title('Comparison of x values')
plt.show()

# Plot y values
plt.figure()
plt.plot(measurements[measurements.columns[1]], label='Predicted y')
plt.plot(validation_df[validation_df.columns[1]], label='Actual y')
plt.xlabel('Time step')
plt.ylabel('y value')
plt.legend()
plt.title('Comparison of y values')
plt.show()

# Plot z values
plt.figure()
plt.plot(measurements[measurements.columns[2]], label='Predicted z')
plt.plot(validation_df[validation_df.columns[2]], label='Actual z')
plt.xlabel('Time step')
plt.ylabel('z value')
plt.legend()
plt.title('Comparison of z values')
plt.show()