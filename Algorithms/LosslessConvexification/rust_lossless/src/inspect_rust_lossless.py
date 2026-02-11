import csv
import matplotlib.pyplot as plt

# File path
csv_file = "trajectory.csv"

# Initialize lists
t, x, y, z = [], [], [], []
vx, vy, vz = [], [], []
mass = []
ux, uy, uz = [], [], []
sigma = []

# New lists for Force calculations
F_sigma = []      # Mass * Sigma (The limit)
F_thrust = []     # Read directly from CSV (The actual force)

# Read CSV
with open(csv_file, newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        # Load variables
        val_t = float(row['t'])
        val_m = float(row['mass'])
        val_ux = float(row['ux'])
        val_uy = float(row['uy'])
        val_uz = float(row['uz'])
        val_sigma = float(row['sigma'])
        
        # Load the new pre-calculated column
        val_thrust_force = float(row['thrust_force'])

        t.append(val_t)
        x.append(float(row['x']))
        y.append(float(row['y']))
        z.append(float(row['z']))
        vx.append(float(row['vx']))
        vy.append(float(row['vy']))
        vz.append(float(row['vz']))
        mass.append(val_m)
        ux.append(val_ux)
        uy.append(val_uy)
        uz.append(val_uz)
        sigma.append(val_sigma)

        # --- ASSIGN FORCES ---
        # 1. Limit: Calculate Mass * Sigma (assuming sigma is accel/throttle bound)
        F_sigma.append(val_m * val_sigma)
        
        # 2. Actual: Use the pre-calculated value from Rust
        F_thrust.append(val_thrust_force)

# Create figure
fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

# 1. Position
axes[0].plot(t, x, label='x')
axes[0].plot(t, y, label='y')
axes[0].plot(t, z, label='z')
axes[0].set_ylabel("Position [m]")
axes[0].legend(loc='upper right')
axes[0].grid(True)

# 2. Velocity
axes[1].plot(t, vx, label='vx')
axes[1].plot(t, vy, label='vy')
axes[1].plot(t, vz, label='vz')
axes[1].set_ylabel("Velocity [m/s]")
axes[1].legend(loc='upper right')
axes[1].grid(True)

# 3. Mass
axes[2].plot(t, mass, label='mass', color='purple')
axes[2].set_ylabel("Mass [kg]")
axes[2].legend(loc='upper right')
axes[2].grid(True)

# 4. Thrust Force (Mass * Sigma vs Pre-calculated Thrust)
axes[3].plot(t, F_sigma, label='Limit (Mass * Sigma)', color='red', linestyle='--', linewidth=2)
axes[3].plot(t, F_thrust, label='Thrust (From CSV)', color='blue', linestyle=':', linewidth=2)

axes[3].set_xlabel("Time [s]")
axes[3].set_ylabel("Thrust Force [N]")
axes[3].legend(loc='upper right')
axes[3].grid(True)
axes[3].set_title("Thrust Profile (Newtons)")

plt.tight_layout()
plt.savefig("trajectory_plot.png", dpi=300)
plt.show()