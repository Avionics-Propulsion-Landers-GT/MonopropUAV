import numpy as np
import csv
import math

class ExtendedKalmanFilter():
    # Quaternions are defined as (real, q1, q2, q3)
    
    def __init__(self, initial_state, initial_measurements, delta_time, q_scalar, r_scalar, initial_p):
        self.state = initial_state
        self.previous_state = initial_state
        self.previous_time = 0 - 2 * delta_time
        self.current_time = 0 - delta_time
        self.delta_time = delta_time
        self.measurement_size = np.size(initial_measurements)

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

        normalized_accel = self.euler_normalize(measurement_prediction[3:6])
        measurement_prediction[3] = normalized_accel[0]
        measurement_prediction[4] = normalized_accel[1]
        measurement_prediction[5] = normalized_accel[2]
        normalized_mag = self.euler_normalize(measurement_prediction[6:9])
        measurement_prediction[6] = normalized_mag[0]
        measurement_prediction[7] = normalized_mag[1]
        measurement_prediction[8] = normalized_mag[2]

        measurement_residual = measurement - measurement_prediction

        # calculate measurement residual covariance
        measurement_residual_covariance = self.measurement_prediction_jacobian() @ self.error_covariance @ self.measurement_prediction_jacobian().T + self.measurement_noise_covariance

        # calculate near-optimal kalman gain
        kalman_gain = self.error_covariance @ self.measurement_prediction_jacobian().T @ np.linalg.inv(measurement_residual_covariance)

        # update state estimate
        self.previous_state = self.state
        self.state = self.quaternion_to_euler(self.quaternion_normalize(self.euler_to_quaternion(self.state + kalman_gain @ measurement_residual)))

        # update covariance estimate
        self.error_covariance = (np.eye(np.size(self.state)) - kalman_gain @ self.measurement_prediction_jacobian()) @ self.error_covariance
        return

    def state_transition_function(self):
        euler_angular_velocity = self.quaternions_to_euler_angular_velocities(self.euler_to_quaternion(self.previous_state), self.euler_to_quaternion(self.state), self.delta_time)
        omega_matrix = np.array([[0, -euler_angular_velocity[0], -euler_angular_velocity[1], -euler_angular_velocity[2]], 
                                 [euler_angular_velocity[0], 0, euler_angular_velocity[2], -euler_angular_velocity[1]], 
                                 [euler_angular_velocity[1], -euler_angular_velocity[2], 0, euler_angular_velocity[0]], 
                                 [euler_angular_velocity[2], euler_angular_velocity[1], -euler_angular_velocity[0], 0]])

        next_state = self.quaternion_normalize(self.euler_to_quaternion(self.state) + 0.5 * self.delta_time * omega_matrix @ self.euler_to_quaternion(self.state))
        return self.quaternion_to_euler(next_state)
    
    def state_transition_jacobian(self):
        return np.eye(np.size(self.state))
    
    def measurement_prediction_function(self):
        euler_angular_velocity = self.get_extrinsic_rotation_matrix(self.state) @ self.quaternions_to_euler_angular_velocities(self.euler_to_quaternion(self.previous_state), self.euler_to_quaternion(self.state), self.delta_time)
        gravity = self.euler_normalize(self.get_extrinsic_rotation_matrix(self.state) @ np.array([0, 0, -9.81]))
        mag = self.euler_normalize(self.get_extrinsic_rotation_matrix(self.state) @ np.array([0.00005, 0, 0]))

        return np.array([euler_angular_velocity[0], euler_angular_velocity[1], euler_angular_velocity[2],
                         gravity[0], gravity[1], gravity[2],
                         mag[0], mag[1], mag[2]])

    def measurement_prediction_jacobian(self):
        jacobian = np.zeros((self.measurement_size, np.size(self.state)))
        jacobian[0][0] = 1
        jacobian[1][1] = 1
        jacobian[2][2] = 1
        return jacobian
    
    def quaternions_to_euler_angular_velocities(self, q1, q2, dt):
        return 2 * self.quaternion_multiply(q2, self.quaternion_inverse(q1))[1:4] / dt

    # euler in the form (x,y,z)
    def euler_to_quaternion(self, euler):
        return np.array([math.cos(euler[0] / 2) * math.cos(euler[1] / 2) * math.cos(euler[2] / 2) + math.sin(euler[0] / 2) * math.sin(euler[1] / 2) * math.sin(euler[2] / 2),
                         math.sin(euler[0] / 2) * math.cos(euler[1] / 2) * math.cos(euler[2] / 2) - math.cos(euler[0] / 2) * math.sin(euler[1] / 2) * math.sin(euler[2] / 2),
                         math.cos(euler[0] / 2) * math.sin(euler[1] / 2) * math.cos(euler[2] / 2) + math.sin(euler[0] / 2) * math.cos(euler[1] / 2) * math.sin(euler[2] / 2),
                         math.cos(euler[0] / 2) * math.cos(euler[1] / 2) * math.sin(euler[2] / 2) - math.sin(euler[0] / 2) * math.sin(euler[1] / 2) * math.cos(euler[2] / 2)])
    
    # quaternion in the form (w,q1,q2,q3) where w is the real rotation component and q1,q2,q3 are the vector components
    def quaternion_to_euler(self, quaternion):
        return np.array([math.atan2(2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]), (1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]))),
                         math.asin(2 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1])),
                         math.atan2(2 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]), (1 - 2 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3])))])
    
    def quaternion_normalize(self, quaternion):
        norm = math.sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3])
        if (norm == 0):
            return np.array([0, 0, 0, 0])
        return quaternion / norm
    
    def euler_normalize(self, euler):
        norm = math.sqrt(euler[0] * euler[0] + euler[1] * euler[1] + euler[2] * euler[2])
        if (norm == 0):
             return np.array([0, 0, 0])
        return euler / norm
    
    def quaternion_inverse(self, quaternion):
        norm = math.sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3])
        if (norm > 0):
            return np.array([quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]]) / norm
        return np.array([1, 0, 0, 0])
    
    def quaternion_multiply(self, q1, q2):
        return np.array([q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
                         q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
                         q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
                         q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]])
    
    #euler in (x,y,z)
    def get_extrinsic_rotation_matrix(self, euler):
        return self.get_extrinsic_z_rotation(euler[2]) @ self.get_extrinsic_y_rotation(euler[1]) @ self.get_extrinsic_x_rotation(euler[0])
    
    def get_extrinsic_x_rotation(self, x):
        return np.array([[1, 0, 0],
                         [0, math.cos(x), -math.sin(x)],
                         [0, math.sin(x), math.cos(x)]])
    
    def get_extrinsic_y_rotation(self, y):
        return np.array([[math.cos(y), 0, math.sin(y)],
                         [0, 1, 0],
                         [-math.sin(y), 0, math.cos(y)]])
    
    def get_extrinsic_z_rotation(self, z):
        return np.array([[math.cos(z), -math.sin(z), 0],
                         [math.sin(z), math.cos(z), 0],
                         [0, 0, 1]])