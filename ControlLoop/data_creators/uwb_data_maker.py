import numpy as np
import pandas as pd
import os

directory_path = '../'  # Set your output directory

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

# --------- UWB Data Generation ---------
def generate_mocked_uwb_data_with_real_anchors(num_points=20001, noise_std=0.00):
    dt = 0.001  # 1 ms timestep
    time = np.round(np.linspace(0, (num_points - 1) * dt, num_points), 4)

    # Real anchor positions (from initAnchors())
    anchors = {
        "A1": np.array([10.0, 10.0, 0.0]),
        "A2": np.array([-10.0, 10.0, 0.0]),
        "A3": np.array([0.0, -14.0, 0.0])
    }

    # Object position: fixed X, Y = 0,0; only Z varies
    obj_xy = np.array([0.0, 0.0])
    z_positions = np.array([trajectory_z(t) for t in time])
    obj_positions = np.column_stack([np.full(num_points, obj_xy[0]),
                                     np.full(num_points, obj_xy[1]),
                                     z_positions])

    # Compute distances
    distances = {}
    for i, (name, anchor_pos) in enumerate(anchors.items(), 1):
        diff = obj_positions - anchor_pos
        dist = np.linalg.norm(diff, axis=1)
        noisy_dist = dist + np.random.normal(0, noise_std, num_points)
        distances[f'Distance{i}'] = noisy_dist

    # Combine into DataFrame
    uwb_data = pd.DataFrame({'Time': time, **distances})
    uwb_data.to_csv(os.path.join(directory_path, 'uwb_combined_distances.csv'), index=False)

# Run it
generate_mocked_uwb_data_with_real_anchors()
