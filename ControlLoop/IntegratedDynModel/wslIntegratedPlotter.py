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

# --- Attitude Plot ---
trace_phi = go.Scatter(x=data['time'], y=data['phi'], mode='lines', name='About x [rad]')
trace_theta = go.Scatter(x=data['time'], y=data['theta'], mode='lines', name='About y [rad]')
trace_psi = go.Scatter(x=data['time'], y=data['psi'], mode='lines', name='Roll (about z) [rad]')

layout_cmd = go.Layout(
    title='Attitude (euler) vs. Time',
    xaxis=dict(title='Time'),
    yaxis=dict(title='Angles'),
    legend_title='Attitude Angles'
)

fig_cmd = go.Figure(data=[trace_phi, trace_theta, trace_psi], layout=layout_cmd)

# --- Desired Position Plot ---
trace_xac = go.Scatter(x=data['time'], y=data['xac'], mode='lines', name='x_desired')
trace_yac = go.Scatter(x=data['time'], y=data['yac'], mode='lines', name='y_desired')
trace_zac = go.Scatter(x=data['time'], y=data['zac'], mode='lines', name='z_desired')

layout_pos_des = go.Layout(
    title='Actual Position vs. Time',
    xaxis=dict(title='Time'),
    yaxis=dict(title='Desired Position')
)

fig_pos_des = go.Figure(data=[trace_xac, trace_yac, trace_zac], layout=layout_pos_des)

# --- Desired Velocity Plot ---
trace_vxac = go.Scatter(x=data['time'], y=data['vxac'], mode='lines', name='vx_desired')
trace_vyac = go.Scatter(x=data['time'], y=data['vyac'], mode='lines', name='vy_desired')
trace_vzac = go.Scatter(x=data['time'], y=data['vzac'], mode='lines', name='vz_desired')

layout_vel_des = go.Layout(
    title='Actual Velocity vs. Time',
    xaxis=dict(title='Time'),
    yaxis=dict(title='Desired Velocity')
)

fig_vel_des = go.Figure(data=[trace_vxac, trace_vyac, trace_vzac], layout=layout_vel_des)

# --- Save HTML files (double write with abspath) ---
position_path = os.path.join(BASE_DIR, "position_plot.html")
velocity_path = os.path.join(BASE_DIR, "velocity_plot.html")
command_path  = os.path.join(BASE_DIR, "command_plot.html")
attitude_path = os.path.join(BASE_DIR, "attitude_plot.html")
pos_des_path  = os.path.join(BASE_DIR, "desired_position_plot.html")
vel_des_path  = os.path.join(BASE_DIR, "desired_velocity_plot.html")

fig_pos.write_html(position_path)
fig_vel.write_html(velocity_path)
fig_cmd.write_html(command_path)
fig_cmd.write_html(attitude_path)
fig_pos_des.write_html(pos_des_path)
fig_vel_des.write_html(vel_des_path)

# Absolute paths
abs_paths = [os.path.abspath(p) for p in [
    position_path, velocity_path, command_path, attitude_path, pos_des_path, vel_des_path
]]
win_paths = [to_windows_path(p) for p in abs_paths]

print("Plots saved and opening in Chrome:")
for path in win_paths:
    print(f"  {path}")
    subprocess.run(["wslview", path])
