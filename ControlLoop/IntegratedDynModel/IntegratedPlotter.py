import pandas as pd
import matplotlib.pyplot as plt
import mpld3
import subprocess

# Read the CSV file. Adjust the filename if needed.
data = pd.read_csv("simulation_results.csv")

# Figure 1: Plot positions (x, y, z) versus time
fig1, ax1 = plt.subplots()
ax1.plot(data['time'], data['x'], label='x')
ax1.plot(data['time'], data['y'], label='y')
ax1.plot(data['time'], data['z'], label='z')
ax1.set_xlabel('Time')
ax1.set_ylabel('Position')
ax1.set_title('Position vs. Time')
ax1.legend()

# Figure 2: Plot velocities (vx, vy, vz) versus time
fig2, ax2 = plt.subplots()
ax2.plot(data['time'], data['vx'], label='vx')
ax2.plot(data['time'], data['vy'], label='vy')
ax2.plot(data['time'], data['vz'], label='vz')
ax2.set_xlabel('Time')
ax2.set_ylabel('Velocity')
ax2.set_title('Velocity vs. Time')
ax2.legend()

# Convert the Matplotlib figures to HTML strings using mpld3.
html_str_pos = mpld3.fig_to_html(fig1)
html_str_vel = mpld3.fig_to_html(fig2)

# Save the HTML strings to files.
with open("position_plot.html", "w") as f:
    f.write(html_str_pos)
with open("velocity_plot.html", "w") as f:
    f.write(html_str_vel)

print("Plots saved to 'position_plot.html' and 'velocity_plot.html'.")

# Now open the HTML files in your browser via 'wslview' (common on WSL)
# If you prefer to use chrome.exe, change the command accordingly.
subprocess.run(["wslview", "position_plot.html"])
subprocess.run(["wslview", "velocity_plot.html"])
