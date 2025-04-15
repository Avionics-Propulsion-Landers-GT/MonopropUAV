import pandas as pd
import plotly.graph_objs as go
import os
import subprocess

# Get base directory
BASE_DIR = os.path.abspath(os.path.dirname(__file__))

# Load the simulation data
csv_path = os.path.join(BASE_DIR, "build/simulation_results.csv")
data = pd.read_csv(csv_path)

# Convert WSL path to Windows-style UNC path for Chrome
def to_windows_path(path, distro="Ubuntu"):
    return f"\\\\wsl.localhost\\{distro}" + path.replace("/", "\\")

# --- Position Plot ---
trace_x = go.Scatter(x=data['time'], y=data['x'], mode='lines', name='x')
trace_y = go.Scatter(x=data['time'], y=data['y'], mode='lines', name='y')
trace_z = go.Scatter(x=data['time'], y=data['z'], mode='lines', name='z')

layout_pos = go.Layout(
    title='Position vs. Time',
    xaxis=dict(title='Time'),
    yaxis=dict(title='Position')
)

fig_pos = go.Figure(data=[trace_x, trace_y, trace_z], layout=layout_pos)

# --- Velocity Plot ---
trace_vx = go.Scatter(x=data['time'], y=data['vx'], mode='lines', name='vx')
trace_vy = go.Scatter(x=data['time'], y=data['vy'], mode='lines', name='vy')
trace_vz = go.Scatter(x=data['time'], y=data['vz'], mode='lines', name='vz')

layout_vel = go.Layout(
    title='Velocity vs. Time',
    xaxis=dict(title='Time'),
    yaxis=dict(title='Velocity')
)

fig_vel = go.Figure(data=[trace_vx, trace_vy, trace_vz], layout=layout_vel)

# --- Thrust & Gimbal Plot ---
trace_thrust = go.Scatter(x=data['time'], y=data['thrust'], mode='lines', name='Thrust [N]')
trace_a = go.Scatter(x=data['time'], y=data['a'], mode='lines', name='Gimbal A [rad]')
trace_b = go.Scatter(x=data['time'], y=data['b'], mode='lines', name='Gimbal B [rad]')

layout_cmd = go.Layout(
    title='Thrust and Gimbal Angles vs. Time',
    xaxis=dict(title='Time'),
    yaxis=dict(title='Commanded Values'),
    legend_title='Command Inputs'
)

fig_cmd = go.Figure(data=[trace_thrust, trace_a, trace_b], layout=layout_cmd)

# --- Save HTML files (double write with abspath) ---
position_path = os.path.join(BASE_DIR, "position_plot.html")
velocity_path = os.path.join(BASE_DIR, "velocity_plot.html")
command_path  = os.path.join(BASE_DIR, "command_plot.html")

fig_pos.write_html(position_path)
fig_vel.write_html(velocity_path)
fig_cmd.write_html(command_path)

# Absolute paths
abs_position_path = os.path.abspath(position_path)
abs_velocity_path = os.path.abspath(velocity_path)
abs_command_path  = os.path.abspath(command_path)

fig_pos.write_html(abs_position_path)
fig_vel.write_html(abs_velocity_path)
fig_cmd.write_html(abs_command_path)

# --- Convert to Windows UNC path ---
win_pos = to_windows_path(abs_position_path)
win_vel = to_windows_path(abs_velocity_path)
win_cmd = to_windows_path(abs_command_path)

print("Plots saved and opening in Chrome:")
print(f"  {win_pos}")
print(f"  {win_vel}")
print(f"  {win_cmd}")

# --- Open in Windows Chrome ---
subprocess.run(["chrome.exe", win_pos])
subprocess.run(["chrome.exe", win_vel])
subprocess.run(["chrome.exe", win_cmd])
