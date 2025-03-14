# Generate UWB trajectory to match the GPS trajectory
import numpy as np
import pandas as pd

"""

    uwb_data_maker.py

    by spencer boebel

    PURPOSE: Run this program to generate a new uwb dataset. You
    can fiddle around with settings like the accel value, standard
    deviation etc. to test the response of the control loop.

"""


# Constants
num_points = 10001  # Total data points
dt = 0.001  # Time step (1 ms)
g = 9.81  # Gravity (m/s^2)

# Initial conditions
x_start, y_start, z_start = 0.0, 0.0, 200.0  # Start position in meters

# Convert acceleration vector to Earth's reference frame
ax, ay, az = 0.1, 0.1, 1.0  # m/s^2 (constant acceleration for 1s)
a_magnitude = 50.0  # m/s^2 (scaled acceleration magnitude)

# Normalize and apply magnitude
a_vector = np.array([ax, ay, az])
a_vector = (a_vector / np.linalg.norm(a_vector)) * a_magnitude  # Normalize and scale

# Velocity and position initialization
velocity = np.array([0.0, 0.0, 0.0])  # Initial velocity (m/s)
position = np.array([x_start, y_start, z_start])  # Initial position

# Store trajectory data
trajectory = np.zeros((num_points, 3))  # Columns: [x, y, z]

for i in range(num_points):
    t = i * dt  # Current time
    
    # Apply acceleration for first 1 second, then only gravity
    if t < 1.0:
        total_acceleration = a_vector - np.array([0, 0, g])  # Include gravity
    else:
        total_acceleration = np.array([0, 0, -g])  # Only gravity after 1s

    # Update velocity and position using kinematics
    velocity += total_acceleration * dt
    position += velocity * dt

    # Store values
    trajectory[i] = [position[0], position[1], position[2]]

# Add Gaussian noise
noise_std = 0.5  # meters
x_noise = np.random.normal(0, noise_std, num_points)
y_noise = np.random.normal(0, noise_std, num_points)

# Apply noise
trajectory[:, 0] += x_noise  # X noise
trajectory[:, 1] += y_noise  # Y noise

# Save to DataFrame
uwb_data_noisy = pd.DataFrame({
    "Time [s]": np.round(np.linspace(0, num_points * dt, num_points), 4),
    "X [m]": trajectory[:, 0],
    "Y [m]": trajectory[:, 1]
})

# Save to new CSV file
csv_uwb_filename = "../../Sensing&Controls/AttitudeEstimation/uwb_data.csv"
uwb_data_noisy.to_csv(csv_uwb_filename, index=False)
