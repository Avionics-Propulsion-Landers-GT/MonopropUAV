# Update to include the full CoP vector (cop_x, cop_y, cop_z)
# CoP_y will depend on beta (sideslip), similar to how CoP_x depends on alpha
# Assume CoP_z is fixed at 0.2 m (approximate axial position of CoP)

import pandas as pd
import numpy as np

cop_z_fixed = 0.2  # meters, static axial CoP position

# Constants
R = 0.02  # Cylinder radius in meters
area = np.pi * R**2  # Frontal area
k = 1.2  # Empirical scaling for CoP lateral shift

# Create log-spaced velocities (avoid log(0))
velocity = np.logspace(np.log10(0.1), np.log10(300), 50)  # 0.1 m/s to 300 m/s

# Create linearly spaced alpha and beta (0 to ~15 degrees in radians)
alpha_rad = np.linspace(0, np.radians(15), 90)
beta_rad = np.linspace(0, np.radians(15), 90)

# Rebuild the data grid with cop_x, cop_y, cop_z
data_full = []
for v in velocity:
    for alpha in alpha_rad:
        for beta in beta_rad:
            # Lateral CoP shifts based on alpha and beta (independently)
            cop_x = k * R * np.sin(alpha)
            cop_y = k * R * np.sin(beta)
            cop_z = cop_z_fixed
            data_full.append([v, alpha, beta, area, cop_x, cop_y, cop_z])

# Convert to DataFrame
df_full = pd.DataFrame(data_full, columns=["velocity", "alpha_rad", "beta_rad", "area", "cop_x", "cop_y", "cop_z"])

# Save the updated CSV file with the full CoP vector
full_file_path = '../Data/state_lookup.csv'
df_full.to_csv(full_file_path, index=False)


