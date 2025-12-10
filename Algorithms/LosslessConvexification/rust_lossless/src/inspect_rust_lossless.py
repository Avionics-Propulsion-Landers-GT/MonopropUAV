import csv
import matplotlib.pyplot as plt

# File path
csv_file = "trajectory.csv"

# Initialize lists to hold data
t, x, y, z = [], [], [], []
vx, vy, vz = [], [], []
mass = []
ux, uy, uz = [], [], []
sigma = []

# Read CSV
with open(csv_file, newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        t.append(float(row['t']))
        x.append(float(row['x']))
        y.append(float(row['y']))
        z.append(float(row['z']))
        vx.append(float(row['vx']))
        vy.append(float(row['vy']))
        vz.append(float(row['vz']))
        mass.append(float(row['mass']))
        ux.append(float(row['ux']))
        uy.append(float(row['uy']))
        uz.append(float(row['uz']))
        sigma.append(float(row['sigma']))

# Create figure and axes
fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

# Position
axes[0].plot(t, x, label='x')
axes[0].plot(t, y, label='y')
axes[0].plot(t, z, label='z')
axes[0].set_ylabel("Position [m]")
axes[0].legend()
axes[0].grid(True)

# Velocity
axes[1].plot(t, vx, label='vx')
axes[1].plot(t, vy, label='vy')
axes[1].plot(t, vz, label='vz')
axes[1].set_ylabel("Velocity [m/s]")
axes[1].legend()
axes[1].grid(True)

# Mass
axes[2].plot(t, mass, label='mass', color='purple')
axes[2].set_ylabel("Mass [kg]")
axes[2].legend()
axes[2].grid(True)

# Thrust and sigma
axes[3].plot(t, ux, label='ux')
axes[3].plot(t, uy, label='uy')
axes[3].plot(t, uz, label='uz')
axes[3].plot(t, sigma, label='sigma', linestyle='--', color='black')
axes[3].set_xlabel("Time step")
axes[3].set_ylabel("Thrust [N] / Sigma")
axes[3].legend()
axes[3].grid(True)

plt.tight_layout()
plt.savefig("trajectory_plot.png", dpi=300)
plt.show()
