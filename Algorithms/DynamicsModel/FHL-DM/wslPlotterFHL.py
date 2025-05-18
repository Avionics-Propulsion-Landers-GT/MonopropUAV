'''MAKE SURE YOU CD INTO THE DIRECTORY THIS FILE SITS IN AND RUN DYNAMICS MODEL BEFORE RUNNING THIS.'''

import pandas as pd
import plotly.graph_objs as go
import os
import subprocess

# Get base directory
BASE_DIR = os.path.abspath(os.path.dirname(__file__))

# Load the simulation data
csv_path = os.path.join(BASE_DIR, "simulation_results.csv")
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

# --- Mass Plot ---
trace_m = go.Scatter(x=data['time'], y=data['m'], mode='lines', name='Mass')

layout_mass = go.Layout(
    title='Mass vs Time',
    xaxis=dict(title='Time'),
    yaxis=dict(title='Mass (kg)')
)

fig_mass = go.Figure(data=[trace_m], layout=layout_mass)

# --- Attitude Plot ---
trace_att_x = go.Scatter(x=data['time'], y=data['att_x'], mode='lines', name='Attitude X')
trace_att_y = go.Scatter(x=data['time'], y=data['att_y'], mode='lines', name='Attitude Y')
trace_att_z = go.Scatter(x=data['time'], y=data['att_z'], mode='lines', name='Attitude Z')
layout_att = go.Layout(
    title='Attitude vs. Time',
    xaxis=dict(title='Time'),
    yaxis=dict(title='Attitude [rad]')
)
fig_att = go.Figure(data=[trace_att_x, trace_att_y, trace_att_z], layout=layout_att)

# --- Thrust & Gimbal Plot ---
trace_thrust = go.Scatter(x=data['time'], y=data['thrust'], mode='lines', name='Thrust [N]')
trace_a = go.Scatter(x=data['time'], y=data['gimbal_a'], mode='lines', name='Gimbal A [rad]')
trace_b = go.Scatter(x=data['time'], y=data['gimbal_b'], mode='lines', name='Gimbal B [rad]')

layout_cmd = go.Layout(
    title='Thrust and Gimbal Angles vs. Time',
    xaxis=dict(title='Time'),
    yaxis=dict(title='Commanded Values'),
    legend_title='Command Inputs'
)

fig_cmd = go.Figure(data=[trace_thrust, trace_a, trace_b], layout=layout_cmd)

# --- RCS Plot --- 
trace_r1 = go.Scatter(x=data['time'], y=data['R1'], mode='lines', name='RC1 Thrust [N]')
trace_r2 = go.Scatter(x=data['time'], y=data['R2'], mode='lines', name='RC2 Thrust [N]')

layout_rcs = go.Layout(
    title='RCS vs.Time',
    xaxis=dict(title='Time'),
    yaxis=dict(title='Commanded Values'),
    legend_title='RCS'
)

fig_rcs = go.Figure(data=[trace_r1, trace_r2], layout=layout_rcs)

# # --- Desired Position Plot ---
# trace_xac = go.Scatter(x=data['time'], y=data['xac'], mode='lines', name='x_desired')
# trace_yac = go.Scatter(x=data['time'], y=data['yac'], mode='lines', name='y_desired')
# trace_zac = go.Scatter(x=data['time'], y=data['zac'], mode='lines', name='z_desired')

# layout_pos_des = go.Layout(
#     title='Actual Position vs. Time',
#     xaxis=dict(title='Time'),
#     yaxis=dict(title='Desired Position')
# )

# fig_pos_des = go.Figure(data=[trace_xac, trace_yac, trace_zac], layout=layout_pos_des)

# # --- Desired Velocity Plot ---
# trace_vxac = go.Scatter(x=data['time'], y=data['vxac'], mode='lines', name='vx_desired')
# trace_vyac = go.Scatter(x=data['time'], y=data['vyac'], mode='lines', name='vy_desired')
# trace_vzac = go.Scatter(x=data['time'], y=data['vzac'], mode='lines', name='vz_desired')

# layout_vel_des = go.Layout(
#     title='Actual Velocity vs. Time',
#     xaxis=dict(title='Time'),
#     yaxis=dict(title='Desired Velocity')
# )

# fig_vel_des = go.Figure(data=[trace_vxac, trace_vyac, trace_vzac], layout=layout_vel_des)

# --- Save HTML files (double write with abspath) ---
position_path = os.path.join(BASE_DIR, "position_plot.html")
velocity_path = os.path.join(BASE_DIR, "velocity_plot.html")
attitude_path = os.path.join(BASE_DIR, "attitude_plot.html")
mass_path = os.path.join(BASE_DIR, "mass_plot.html")
command_path  = os.path.join(BASE_DIR, "command_plot.html")
rcs_path = os.path.join(BASE_DIR, "rcs_plot.html")
# pos_des_path  = os.path.join(BASE_DIR, "desired_position_plot.html")
# vel_des_path  = os.path.join(BASE_DIR, "desired_velocity_plot.html")

fig_pos.write_html(position_path)
fig_vel.write_html(velocity_path)
fig_att.write_html(attitude_path)
fig_mass.write_html(mass_path)
fig_cmd.write_html(command_path)
fig_rcs.write_html(rcs_path)
# fig_pos_des.write_html(pos_des_path)
# fig_vel_des.write_html(vel_des_path)

# Absolute paths
abs_paths = [os.path.abspath(p) for p in [
    position_path, velocity_path, attitude_path, mass_path, command_path, rcs_path
]]
win_paths = [to_windows_path(p) for p in abs_paths]

print("Plots saved and opening in Chrome:")
for path in win_paths:
    print(f"  {path}")
    subprocess.run(["wslview", path])
