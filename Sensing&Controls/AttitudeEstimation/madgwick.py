import time
import numpy as np

#   MADGWICK FILTER EXPLANATION
#################################################################################################################################################
#                                                                                                                                               #
#   Calculates the object orientation accurately by 3-axis of accelerometer, 3-axis of gyroscope, and 3-axis of magnetometer.                   #
#   The filter using quaternion as orientation representation to describe the object orientation in 3D world                                    #
#   The objective of Madgwick filter is to minimize the error created by raw gyroscope data, which always include high frequency of noise.      # 
#   Therefore, accelerometer and magnetometer used to compute the gyroscope measurement error and correct it via gradient-descent algorithm.    #
#                                                                                                                                               #
#   Algorithm:                                                                                                                                  #
#       1) Read sensor data to obtain sensor measurements.                                                                                      #
#       2) Compute oritentation increment from accelerometer and magnetometer measurements (gradient step).                                     #
#       3) Compute orientation from gyroscope measurements (numeriacal integration).                                                            #
#       4) Fuse the measurements from the gradient step and numerical integration to obtain estimated attitude.                                 #
#                                                                                                                                               #
#################################################################################################################################################

# A class for the complementary filter for attitude estimation
class Madgwick():
    # Constructor for filter that sets the gain and beta. Both values should be in the range [0,1]
    # Gain adjusts the magnitude of the gyroscope compensation applied during quaternion integration
    # Beta controls the weight given to the gradient descent correction step.
    def __init__(self, gain,beta):
        self.gain = gain
        self.beta = beta
        self.last_update_time = -1
        self.attitude_estimation = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) # Initial quaternion (no rotation)
    
    # Initializes filter with a start time and initial attitude estimation
    def initialize(self, time, initial_attitude_estimation):
        self.last_update_time = time
        if(np.linalg.norm(initial_attitude_estimation) != 0):
            self.attitude_estimation = initial_attitude_estimation / np.linalg.norm(initial_attitude_estimation)

    def update(self, update_arr):
        # Read time data, calculate delta time, and update previous update time
        dt = update_arr[0] - self.last_update_time
        self.last_update_time = update_arr[0]

        # Read accelerometer, gyrometer, and magnometer data
        accel = np.array([update_arr[1], update_arr[2], update_arr[3]])
        gyro = np.array([update_arr[4], update_arr[5], update_arr[6]])
        mag = np.array([update_arr[7], update_arr[8], update_arr[9]])

        # Apply Magwick Filter
        self.q = self.madgwick_update(self.q, gyro, accel, mag, dt)

        # Calculates Attitude in euler angles using quaternion conversion formulas
        # Source: https://madecalculators.com/quaternion-to-euler-calculator/
        roll = np.arctan2(2*(self.q[0]*self.q[1] + self.q[2]*self.q[3]), 1 - 2*(self.q[1]**2 + self.q[2]**2))
        pitch = np.arcsin(2*(self.q[0]*self.q[2] - self.q[3]*self.q[1]))
        yaw = np.arctan2(2*(self.q[0]*self.q[3] + self.q[1]*self.q[2]), 1 - 2*(self.q[2]**2 + self.q[3]**2))

        self.attitude_estimation = np.array([roll, pitch, yaw])
        
        # Returns the attitude estimate
        return self.get_estimate()
 

    def get_estimate(self):
        return self.attitude_estimation

    # Normalize a vector
    def normalize(self, vector):
        return vector / np.linalg.norm(vector)

    # Performs algorithm of the Madgwick filter
    def madgwick_update(self, q, gyro, accel, mag, dt):
        # Normalize accelorometer and magnometer vectors
        accel = self.normalize(accel)
        mag = self.normalize(mag)

        # Gradient descent step to minimize the error
        step = self.compute_gradient(q, accel, mag)
        step = self.normalize(step)
        stepQ = np.array([step.T[0], step.T[1], step.T[2], step.T[3]])

        ## Integrate Gyroscope
        # Gyroscope compensation drift
        gyroQ = np.array([0,*gyro]) + self.quaternion_multiply(np.array([q[0], -q[1], -q[2], -q[3]]), stepQ) * 2 * dt * self.gain * -1

        # Compute rate of change of quaternion
        qdot = self.quaternion_multiply(q,gyroQ) * 0.5 - self.beta * step.T

        # Integrate and yield quaternion
        q += qdot * dt
        return self.normalize(q)

    # Computes the gradient of the error function for the Madgwick filter
    def compute_gradient(self, q, accel, mag):
        # References direction of Earth's magnetic field
        # h = self.quaternion_multiply(self.quaternion_multiply(q,np.array([0,*mag])),
        #     np.array([q[0], -q[1], -q[2], -q[3]])
        #     )
        h = self.quaternion_multiply(self.quaternion_multiply(np.array([q[0], -q[1], -q[2], -q[3]]),np.array([0,*mag])),
            q
            )
         
        b = np.array([0,np.linalg.norm(h[1:3]), 0, h[3]])


        # Calculates objective function and the jacobian of the objective function respectively
        # Formulas sourced from: https://medium.com/@k66115704/imu-madgwick-filter-explanation-556fbe7f02e3
        f = np.array([
            2 * (q[1] * q[3] - q[0] * q[2]) - accel[0],
            2 * (q[0] * q[1] + q[2] * q[3]) - accel[1],
            2 * (0.5 - q[1]**2 - q[2]**2) - accel[2],
            2 * b[1] * (0.5 - q[2]**2 - q[3]**2) + 2 * b[3] * (q[1] * q[3] - q[0] * q[2]) - mag[0],
            2 * b[1] * (q[1] * q[2] - q[0] * q[3]) + 2 * b[3] * (q[0] * q[1] + q[2] * q[3]) - mag[1],
            2 * b[1] * (q[0] * q[2] + q[1] * q[3]) + 2 * b[3] * (0.5 - q[1]**2 - q[2]**2) - mag[2]
        ])
        J = np.array([
            [-2 * q[2], 2 * q[3], -2 * q[0], 2 * q[1]],
            [2 * q[1], 2 * q[0], 2 * q[3], 2 * q[2]],
            [0, -4 * q[1], -4 * q[2], 0],
            [-2 * b[3] * q[2],  2 * b[3] * q[3], -4 * b[1] * q[2] - 2 * b[3] * q[0], -4 * b[1] * q[3] + 2 * b[3] * q[1]],
            [-2 * b[1] * q[3] + 2 * b[3] * q[1],  2 * b[1] * q[2] + 2 * b[3] * q[0],  2 * b[1] * q[1] + 2 * b[3] * q[3], -2 * b[1] * q[0] + 2 * b[3] * q[2]],
            [ 2 * b[1] * q[2],  2 * b[1] * q[3] - 4 * b[3] * q[1],  2 * b[1] * q[0] - 4 * b[3] * q[2],  2 * b[1] * q[1]]
        ])

        gradient = J.T @ f
        return gradient

    # Quaternion multiplication helper function
    def quaternion_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return np.array([w, x, y, z])

