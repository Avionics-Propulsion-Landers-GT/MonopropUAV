# Re-run the inertia generation after environment reset
import numpy as np
import pandas as pd

# Box properties (assumed)
m = 0.05  # kg
w, h, d = 0.1, 0.1, 0.2  # width, height, depth in meters

# Compute the local box inertia tensor (about its own CoM)
I_xx = (1/12) * m * (h**2 + d**2)
I_yy = (1/12) * m * (w**2 + d**2)
I_zz = (1/12) * m * (w**2 + h**2)
I_box = np.array([[I_xx, 0, 0],
                  [0, I_yy, 0],
                  [0, 0, I_zz]])

# Gimbal angles range
angles = np.linspace(-1, 1, 50)  # -1 rad to +1 rad

# Storage for A and B component CSVs
rows_A, rows_B = [], []

for angle in angles:
    # Rotation for Component A (around Y-axis)
    R_y = np.array([[np.cos(angle), 0, np.sin(angle)],
                    [0, 1, 0],
                    [-np.sin(angle), 0, np.cos(angle)]])
    I_A_body = R_y @ I_box @ R_y.T
    row_A = [angle] + I_A_body.flatten(order='C').tolist()
    rows_A.append(row_A)

    # Rotation for Component B (around X-axis)
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(angle), -np.sin(angle)],
                    [0, np.sin(angle), np.cos(angle)]])
    I_B_body = R_x @ I_box @ R_x.T
    row_B = [angle] + I_B_body.flatten(order='C').tolist()
    rows_B.append(row_B)

# Create DataFrames
cols = ["angle_rad"] + [f"MoIM_{i}{j}" for i in range(3) for j in range(3)]
df_A = pd.DataFrame(rows_A, columns=cols)
df_B = pd.DataFrame(rows_B, columns=cols)

# Save CSVs
file_A = '../control_lookup_a.csv'
file_B = '../control_lookup_b.csv'
df_A.to_csv(file_A, index=False)
df_B.to_csv(file_B, index=False)

file_A, file_B
