import time
import numpy as np

#   MADGWICK FILTER EXPLANATION
#############################################################################################################################################
#                                                                                                                                           #
#   Calculates the object orientation accurately by 3-axis of accelerometer, 3-axis of gyroscope, and 3-axis of magnetometer(Optional).     #
#   The filter using quaternion as orientation representation to describe the object orientation in 3D world                                #
#   The objective of Madgwick filter is to minimize the error created by raw gyroscope data, which always include high frequency of noise.  # 
#   Therefore, accelerometer and magnetometer used to compute the gyroscope measurement error and correct it via gradient-descent algorithm.#
#                                                                                                                                           #
#   Algorithm:                                                                                                                              #
#       1) Read sensor data to obtain sensor measurements.                                                                                  #
#       2) Compute oritentation increment from accelerometer measurements (gradient step).                                                  #
#       3) Compute orientation from gyroscope measurements (numeriacal integration).                                                        #
#       4) Fuse the measurements from both the accelerometer and gyroscope to obtain estimated altitude.                                    #
#                                                                                                                                           #
#############################################################################################################################################

# Initialize variables
q = np.array([1.0, 0.0, 0.0, 0.0]) # Initial quaternion (no rotation)

beta = 0.1 # The gain of the Madgwick filter
dt = 0.01  # Time step (e.g., 10 ms)

while True:
    # Read sensor data
    accel = read_accelerometer()
    gyro = read_gyroscope()

    # Apply Magwick Filter
    q = madgwick_update(q, gyro, accel, beta, dt)

    # Calculate roll, pitch, yaw from quaternion
    roll = np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2))
    pitch = np.arcsin(2*(q[0]*q[2] - q[3]*q[1]))
    yaw = np.arctan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]**2 + q[3]**2))

    # Print the estimated orientation
    print(f"Roll: {np.degrees(roll):.2f}, Pitch: {np.degrees(pitch):.2f}, Yaw: {np.degrees(yaw):.2f}")

    # Wait for the next time step
    time.sleep(dt)

# Normalize a vector
def normalize(vector):
    return vector / np.linalg.norm(vector)

# Gyroscope integration (approximate quaternion derivative)
def integrate_gyro(q, gyro, dt):
    gx, gy, gz = gyro
    origin_q = np.array([0, gx, gy, gz])
    quat_diff = quaternion_multiply(q, origin_q)
    quat_change = 0.5 * quat_diff
    quat_new = q + quat_change * dt
    return normalize(quat_new)

# Madgwick filter update function
def madgwick_update(q, gyro, accel, beta, dt):
    # Normalize accelerometer measurement
    accel = normalize(accel)

    # Estimated direction of gravity
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    gravity = quaternion_multiply(quaternion_multiply(q, np.array([0, 0, 0, 1])), q_conj)[1:]

    # Error is the cross product between estimated and measured direction of gravity
    error = np.cross(gravity, accel)
    
    # Apply feedback to correct gyroscope bias
    gyro_corrected = gyro + beta * error

    # Integrate gyroscope to update quaternion
    q = integrate_gyro(q, gyro_corrected, dt)

    return q

# Quaternion multiplication helper function
def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])
