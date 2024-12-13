import numpy as np

class Mahony:
    # Mahony Filter implementation for attitude estimation
    # This filter combines sensor data (accelerometer, gyroscope, and magnetometer)
    # to estimate the orientation of an object in space.

    def __init__(self, Kp, Ki=0):
        # Constructor to initialize the Mahony filter parameters
        # Kp: Proportional gain, adjusts response to errors
        # Ki: Integral gain, helps reduce steady-state errors (optional, default is 0)

        self.Kp = Kp  # Proportional gain for error correction
        self.Ki = Ki  # Integral gain for accumulated error correction
        self.last_update_time = -1  # Tracks the last update timestamp
        self.attitude_estimation = np.zeros(3)  # Estimated attitude vector (3D)
        self.integral_error = np.zeros(3)  # Accumulated error for integral term

    def initialize(self, time, initial_attitude_estimation):
        # Initialize the filter with the starting time and attitude
        # time: Initial timestamp (e.g., in seconds)
        # initial_attitude_estimation: Initial orientation vector (3D)

        self.last_update_time = time  # Store the initial time

        # Normalize the initial attitude estimation vector (if non-zero)
        if np.linalg.norm(initial_attitude_estimation) != 0:
            self.attitude_estimation = initial_attitude_estimation / np.linalg.norm(initial_attitude_estimation)

    def update(self, update_arr):
        # Update the filter with new sensor data
        # update_arr: Array containing time, accelerometer, gyroscope, and magnetometer data
        # Format: [time, ax, ay, az, gx, gy, gz, mx, my, mz]

        # Calculate time difference since the last update
        delta_t = update_arr[0] - self.last_update_time
        self.last_update_time = update_arr[0]  # Update the last timestamp

        # Extract sensor data from the input array
        accel = np.array(update_arr[1:4])  # Accelerometer readings (x, y, z)
        gyro = np.array(update_arr[4:7])  # Gyroscope readings (x, y, z)
        mag = np.array(update_arr[7:10])  # Magnetometer readings (x, y, z)

        # Normalize accelerometer data to ensure its magnitude is 1
        if np.linalg.norm(accel) != 0:
            accel /= np.linalg.norm(accel)

        # Normalize magnetometer data to ensure its magnitude is 1
        if np.linalg.norm(mag) != 0:
            mag /= np.linalg.norm(mag)

        # Calculate the error between the estimated and measured direction of gravity
        # This helps align the estimated attitude with accelerometer data
        v = np.cross(self.attitude_estimation, accel)
        
        # Calculate the error between the estimated and measured direction of the magnetic field
        # This helps align the estimated attitude with magnetometer data
        w = np.cross(self.attitude_estimation, mag)
        
        # Combine gravity and magnetic errors for a unified correction term
        error = v + w

        # Update the integral error term based on the accumulated error and time step
        self.integral_error += error * delta_t

        # Apply proportional and integral feedback to compute the correction
        feedback = self.Kp * error + self.Ki * self.integral_error

        # Update the estimated attitude using gyroscope data and feedback correction
        rate_of_change = gyro + feedback  # Combined rate of change
        self.attitude_estimation += rate_of_change * delta_t

        # Normalize the updated attitude estimation to maintain unit magnitude
        if np.linalg.norm(self.attitude_estimation) != 0:
            self.attitude_estimation /= np.linalg.norm(self.attitude_estimation)

        # Return the current estimated attitude
        return self.get_estimate()

    def get_estimate(self):
        # Retrieve the current attitude estimation
        # Returns the estimated orientation vector (3D)
        return self.attitude_estimation
