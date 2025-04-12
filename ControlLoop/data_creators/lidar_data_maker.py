import numpy as np
import pandas as pd
import os

# Time settings
num_points = 20001
dt = 0.001  # 1 ms timestep
time = np.round(np.linspace(0, (num_points - 1) * dt, num_points), 4)

# Noise settings
noise_std = 0.01  # meters

# Trajectory function for Z
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

# Generate Z values based on trajectory + noise
z = np.array([trajectory_z(t) for t in time])
z += np.random.normal(0, noise_std, num_points)

# Combine and save
lidar_df = pd.DataFrame({
    "time": time,
    "z": z
})

lidar_df.to_csv(f'../Data/lidar_data.csv', index=False)
