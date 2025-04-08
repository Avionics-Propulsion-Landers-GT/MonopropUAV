import numpy as np
import pandas as pd

# Parameters
num_points = 20001
dt = 0.001
time = np.round(np.linspace(0, (num_points - 1) * dt, num_points), 6)

# Set random seed for consistency
np.random.seed(42)

# Near-zero angular velocities (rad/s) with tiny Gaussian noise
gyro_noise_std = 0.005
gyro = np.random.normal(0, gyro_noise_std, size=(num_points, 3))
gyro[:, 1] += 0.2 # Add some directional noise to gyro 

# Accelerometer: mostly gravity in Z, tiny noise in all axes
accel_noise_std = 0.02
accel = np.random.normal(0, accel_noise_std, size=(num_points, 3))
accel[:, 2] += 9.81  # Add gravity to Z

# Magnetometer: constant small field, maybe pointing North
mag = np.full((num_points, 3), [0.0000, 0.0, 0.0])

# Assemble DataFrame
imu_data = pd.DataFrame({
    "time": time,
    "x ang vel": gyro[:, 0],
    "y ang vel": gyro[:, 1],
    "z ang vel": gyro[:, 2],
    "x accel": accel[:, 0],
    "y accel": accel[:, 1],
    "z accel": accel[:, 2],
    "x mag": mag[:, 0],
    "y mag": mag[:, 1],
    "z mag": mag[:, 2]
})

# Save to CSV
imu_data.to_csv("../imu_data.csv", index=False)
