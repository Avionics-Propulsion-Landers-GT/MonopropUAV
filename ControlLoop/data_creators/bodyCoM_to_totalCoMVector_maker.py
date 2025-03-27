# Re-run the CoM shift CSV generation after environment reset
import numpy as np
import pandas as pd

# Define gimbal angle range
angles = np.linspace(-1, 1, 50)  # -1 rad to +1 rad

# Assumptions for analytic "best guess"
m_body = 0.7  # kg, static mass of the rocket body
m_tvc = 0.05    # kg, each TVC component mass

# Distance from the body CoM to each TVC element mounting point (fixed)
d_tvc_mount = 0.1  # meters along -Z (rear)

# TVC arm length (how far the TVC's mass shifts when gimbaled)
tvc_arm = 0.01  # meters

# Storage for CSV rows
data = []

for a in angles:
    for b in angles:
        # Estimate the TVC CoM shift due to gimbal angles (small-angle linear approx for now)
        # TVC A swings in XZ plane (by angle a), TVC B in YZ plane (by angle b)
        r_tvc_a_x = m_tvc * tvc_arm * np.sin(a)
        r_tvc_a_y = 0
        r_tvc_a_z = m_tvc * (d_tvc_mount - tvc_arm * (1 - np.cos(a)))

        r_tvc_b_x = 0
        r_tvc_b_y = m_tvc * tvc_arm * np.sin(b)
        r_tvc_b_z = m_tvc * (d_tvc_mount - tvc_arm * (1 - np.cos(b)))

        # Total mass
        m_total = m_body + 2 * m_tvc

        # Compute total CoM shift vector (x, y, z) relative to body CoM
        CoM_x = (r_tvc_a_x + r_tvc_b_x) / m_total
        CoM_y = (r_tvc_a_y + r_tvc_b_y) / m_total
        CoM_z = (r_tvc_a_z + r_tvc_b_z) / m_total  # this would shift slightly aft

        data.append([a, b, CoM_x, CoM_y, CoM_z])

# Create DataFrame and export
df_com = pd.DataFrame(data, columns=["a_rad", "b_rad", "CoM_x", "CoM_y", "CoM_z"])
csv_path = '../system_com_shift.csv'
df_com.to_csv(csv_path, index=False)

csv_path
