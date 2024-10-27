# Anyi Lin
import numpy as np

# A class for the complementary filter for attitude estimation
class Complementary():
    
    # Constructor for filter that sets the gain. The gain should be in the range [0, 1]. 
    def __init__(self, gain):
        self.gain = gain
        self.last_update_time = -1
        self.attitude_estimation = np.zeros(3)

    # Initializes filter with a start time and initial attitude estimation
    def initialize(self, time, initial_attitude_estimation):
        self.last_update_time = time
        if (np.linalg.norm(initial_attitude_estimation) != 0):
            self.attitude_estimation = initial_attitude_estimation / np.linalg.norm(initial_attitude_estimation)

    # Runs an update of the filter, estimating the attitude with all 3 sensors and then averaging them
    def update(self, update_arr):
        # Gets and formats data from the input array
        delta_t = update_arr[0] - self.last_update_time
        self.last_update_time = update_arr[0]
        accel = np.array([update_arr[1], update_arr[2], update_arr[3]])
        gyro = np.array([update_arr[4], update_arr[5], update_arr[6]])
        mag =  np.array([update_arr[7], update_arr[8], update_arr[9]])

        # Calculates x and y tilt from accelerometer, adjusts magnetometer data with tilt angles, then calculates
        # z tilt from adjusted magnetometer data
        tilt_angles = np.array([np.arctan2(accel[1], accel[0]), np.arctan2(-accel[0], np.sqrt(accel[1] ** 2 + accel[2] ** 2)), 0])
        b = np.matmul(np.array(
            [[np.cos(tilt_angles[0]), np.sin(tilt_angles[0]) * np.sin(tilt_angles[1]), np.sin(tilt_angles[0]) * np.cos(tilt_angles[1])],
            [0, np.cos(tilt_angles[1]), -np.sin(tilt_angles[1])],
            [-np.sin(tilt_angles[0]), np.cos(tilt_angles[0]) * np.sin(tilt_angles[1]), np.cos(tilt_angles[0]) * np.cos(tilt_angles[1])]]), mag)
        tilt_angles[2] = np.arctan2(-b[1], b[0])

        # Estimates attitude from previous estimate and gyro data with simple integration over time
        gyro_estimate = np.array([
            self.attitude_estimation[0] + gyro[0] * delta_t,
            self.attitude_estimation[1] + gyro[1] * delta_t,
            self.attitude_estimation[2] + gyro[2] * delta_t
            ])
        
        # Takes an average of the two estimates and normalizes it
        self.attitude_estimation =  self.gain * gyro_estimate + (1 - self.gain) * tilt_angles
        if (np.linalg.norm(self.attitude_estimation) != 0):
            self.attitude_estimation = self.attitude_estimation / np.linalg.norm(self.attitude_estimation)

        # Returns the attitude estimate
        return self.get_estimate()


    # Returns the attitude estimate
    def get_estimate(self):
        return self.attitude_estimation
        