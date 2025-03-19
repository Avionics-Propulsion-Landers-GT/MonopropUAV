# Re-run the code after execution state reset

import pandas as pd
import numpy as np

"""

    gps_data_maker.py

    by spencer boebel

    PURPOSE: Run this program to generate a new gps dataset. You
    can fiddle around with settings like the accel value, standard
    deviation etc. to test the response of the control loop.

"""

# Constants
num_points = 10001  # Total data points
dt = 0.001  # Time step (1 ms)
g = 9.81  # Gravity (m/s^2)

# Initial conditions
lat_start = 33.0  # degrees north
lon_start = -85.0  # degrees west
alt_start = 200.0  # meters above MSL

# Convert acceleration vector to Earth's reference frame
ax, ay, az = 0.1, 0.1, 1.0  # m/s^2 (constant acceleration for 1s)
a_magnitude = 50.0  # m/s^2 (scaled acceleration magnitude)

# Normalize and apply magnitude
a_vector = np.array([ax, ay, az])
a_vector = (a_vector / np.linalg.norm(a_vector)) * a_magnitude  # Normalize and scale

# Velocity and position initialization
velocity = np.array([0.0, 0.0, 0.0])  # Initial velocity (m/s)
position = np.array([0.0, 0.0, alt_start])  # Initial position (x, y in meters, alt in meters)

# Store trajectory data
trajectory = np.zeros((num_points, 3))  # Columns: [lat, lon, alt]

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

    # Convert (x, y) from meters to latitude/longitude
    lat = lat_start + (position[1] / 111320)  # Latitude conversion
    lon = lon_start + (position[0] / (111320 * np.cos(np.radians(lat_start))))  # Longitude conversion

    # Store values
    trajectory[i] = [lat, lon, position[2]]

# Add Gaussian noise
noise_std = 3.0  # meters
lat_noise_std = noise_std / 111320
lon_noise_std = noise_std / (111320 * np.cos(np.radians(lat_start)))
alt_noise_std = noise_std

lat_noise = np.random.normal(0, lat_noise_std, num_points)
lon_noise = np.random.normal(0, lon_noise_std, num_points)
alt_noise = np.random.normal(0, alt_noise_std, num_points)

# Apply noise
trajectory[:, 0] += lat_noise
trajectory[:, 1] += lon_noise
trajectory[:, 2] += alt_noise

# Save to DataFrame
gps_data_noisy = pd.DataFrame({
    "Latitude [deg]": trajectory[:, 0],
    "Longitude [deg]": trajectory[:, 1],
    "Altitude [m above MSL]": trajectory[:, 2]
})

# Save to new CSV file
csv_noisy_filename =  "../../Sensing&Controls/AttitudeEstimation/gps_data.csv"
gps_data_noisy.to_csv(csv_noisy_filename, index=False)
