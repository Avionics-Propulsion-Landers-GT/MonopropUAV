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

# --------- Constants ---------
num_points = 20001  # Total data points for 20 seconds @ 1ms step
dt = 0.001  # Time step (1 ms)

# Static location for simplicity
lat_start = 33.0  # degrees north
lon_start = -85.0  # degrees west
alt_start = 200.0  # meters above MSL (base altitude)

# --------- Trajectory Function (Z-axis only) ---------
def trajectory_z(t):
    pi = np.pi
    if 0.0 <= t < 5.0:
        return -0.5 * np.cos(pi * t / 5.0) + 0.5
    elif 5.0 <= t < 10.0:
        return 1.0
    elif 10.0 <= t < 15.0:
        return 0.5 * np.cos(pi * t / 5.0) + 0.5
    elif 15.0 <= t <= 20.0:
        return 0.0
    else:
        return 1.0

# --------- Generate GPS Data ---------
time = np.round(np.linspace(0, (num_points - 1) * dt, num_points), 4)
altitude = np.array([alt_start + trajectory_z(t) for t in time])

# Add Gaussian noise
noise_std = 3.0  # meters
lat_noise_std = noise_std / 111320
lon_noise_std = noise_std / (111320 * np.cos(np.radians(lat_start)))
alt_noise_std = noise_std

lat_noise = np.random.normal(0, lat_noise_std, num_points)
lon_noise = np.random.normal(0, lon_noise_std, num_points)
alt_noise = np.random.normal(0, alt_noise_std, num_points)

# Apply noise
latitudes = lat_start + lat_noise
longitudes = lon_start + lon_noise
altitudes = altitude + alt_noise

# --------- Save to CSV ---------
gps_data_noisy = pd.DataFrame({
    "Latitude [deg]": latitudes,
    "Longitude [deg]": longitudes,
    "Altitude [m above MSL]": altitudes
})

csv_noisy_filename = "../gps_data.csv"
gps_data_noisy.to_csv(csv_noisy_filename, index=False)
