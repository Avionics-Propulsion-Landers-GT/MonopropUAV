import numpy as np
import csv
import math

class ExtendedKalmanFilterAltitude():
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
        self.previous_time = self.current_time
        self.current_time = data[0]
        self.delta_time = self.current_time - self.previous_time
        return np.array([data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9]])

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
        self.previous_velocity = self.velocity
        # update state estimate
        self.previous_state = self.state
        self.state = self.state + kalman_gain @ measurement_residual

        # update covariance estimate
        self.error_covariance = (np.eye(np.size(self.state)) - kalman_gain @ self.measurement_prediction_jacobian()) @ self.error_covariance
        return

    def state_transition_function(self):
        self.velocity = (self.state - self.previous_state) / self.delta_time
        return self.state + self.velocity*self.delta_time
    
    def state_transition_jacobian(self):
        acceleration = (self.velocity - self.previous_velocity) / self.delta_time
        jacobian = np.array([1 + (acceleration/self.velocity)])
    
    def measurement_prediction_function(self):
        return self.state
    def measurement_prediction_jacobian(self):
        jacobian = np.ones((self.measurement_size, np.size(self.state)))
        return jacobian
