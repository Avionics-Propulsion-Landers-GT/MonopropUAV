import numpy as np
import csv
import math

class ExtendedKalmanFilterXY():
    # Quaternions are defined as (real, q1, q2, q3)
    
    def __init__(self, initial_state, initial_measurements, delta_time, q_scalar, r_scalar, initial_p):
        self.state = initial_state
        self.previous_state = initial_state
        self.previous_time = 0 - 2 * delta_time
        self.current_time = 0 - delta_time
        self.delta_time = delta_time
        self.measurement_size = np.size(initial_measurements)
        self.previous_velocity = 0
        self.velocity = 0

        self.process_noise_covariance = np.eye(np.size(self.state)) * q_scalar
        self.measurement_noise_covariance = np.eye(self.measurement_size) * r_scalar

        self.error_covariance = np.eye((np.size(self.state))) * initial_p

    def parse_data(self, data):

        if not hasattr(self, 'lat0'):
            self.lon0, self.lat0 = data[0], data[1]
    
        # Convert latitude/longitude to local x, y in meters
        x = (data[0] - self.lon0) * np.cos(np.radians(self.lat0)) * 111320
        y = (data[1] - self.lat0) * 111320

        self.previous_time = self.current_time
        self.current_time = self.current_time + self.delta_time
        self.delta_time = self.current_time - self.previous_time
        return np.array([x, y])

    def predict(self):
        # Update state estimate
        self.previous_state = self.state
        self.state = self.state_transition_function()

        # Predict covariance estimate
        self.error_covariance = self.state_transition_jacobian() @ self.error_covariance @ self.state_transition_jacobian().T + self.process_noise_covariance
        return
    
    def update(self, data):
        measurement = self.parse_data(data)

        # calculate measurement residual
        measurement_prediction = self.measurement_prediction_function()

        measurement_residual = measurement - measurement_prediction

        # calculate measurement residual covariance
        measurement_residual_covariance = self.measurement_prediction_jacobian() @ self.error_covariance @ self.measurement_prediction_jacobian().T + self.measurement_noise_covariance

        # calculate near-optimal kalman gain
        kalman_gain = self.error_covariance @ self.measurement_prediction_jacobian().T @ np.linalg.inv(measurement_residual_covariance)

        # update state estimate
        self.previous_state = self.state
        self.state = self.state + kalman_gain @ measurement_residual

        # update covariance estimate
        self.error_covariance = (np.eye(np.size(self.state)) - kalman_gain @ self.measurement_prediction_jacobian()) @ self.error_covariance
        return

    def state_transition_function(self):
        dt = self.delta_time
        x, y, vx, vy = self.state
        
        # Update position using velocity
        x_new = x + vx * dt
        y_new = y + vy * dt

        # Keep velocity the same (constant velocity model)
        return np.array([x_new, y_new, vx, vy])
    
    def state_transition_jacobian(self):
        dt = self.delta_time
        return np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt], 
            [0, 0,  1,  0], 
            [0, 0,  0,  1]   
        ])
    
    def measurement_prediction_function(self):
        x, y, _, _ = self.state
        return np.array([x, y]) 
    
    def measurement_prediction_jacobian(self):
        return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0] 
        ])
