import pandas as pd
import plotly.graph_objs as go
import os
import subprocess

# Get base directory
BASE_DIR = os.path.abspath(os.path.dirname(__file__))

# Load the simulation data
csv_path = os.path.join(BASE_DIR, "build/simulation_results.csv")
data = pd.read_csv(csv_path)

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

# --- Command Plot (Thrust and Gimbal) ---
trace_thrust = go.Scatter(x=data['time'], y=data['thrust'], mode='lines', name='Thrust [N]')
trace_a = go.Scatter(x=data['time'], y=data['a'], mode='lines', name='Gimbal A [rad]')
trace_b = go.Scatter(x=data['time'], y=data['b'], mode='lines', name='Gimbal B [rad]')

layout_cmd = go.Layout(
    title='Commanded Thrust and Gimbal Angles',
    xaxis=dict(title='Time'),
    yaxis=dict(title='Value'),
    legend_title='Commanded Inputs'
)

fig_cmd = go.Figure(data=[trace_thrust, trace_a, trace_b], layout=layout_cmd)

# --- Desired Position Plot (xac, yac, zac) ---
trace_xac = go.Scatter(x=data['time'], y=data['xac'], mode='lines', name='x_des')
trace_yac = go.Scatter(x=data['time'], y=data['yac'], mode='lines', name='y_des')
trace_zac = go.Scatter(x=data['time'], y=data['zac'], mode='lines', name='z_des')

layout_pos_des = go.Layout(
    title='Desired Position vs. Time',
    xaxis=dict(title='Time'),
    yaxis=dict(title='Desired Position')
)

fig_pos_des = go.Figure(data=[trace_xac, trace_yac, trace_zac], layout=layout_pos_des)

# --- Desired Velocity Plot (vxac, vyac, vzac) ---
trace_vxac = go.Scatter(x=data['time'], y=data['vxac'], mode='lines', name='vx_des')
trace_vyac = go.Scatter(x=data['time'], y=data['vyac'], mode='lines', name='vy_des')
trace_vzac = go.Scatter(x=data['time'], y=data['vzac'], mode='lines', name='vz_des')

layout_vel_des = go.Layout(
    title='Desired Velocity vs. Time',
    xaxis=dict(title='Time'),
    yaxis=dict(title='Desired Velocity')
)

fig_vel_des = go.Figure(data=[trace_vxac, trace_vyac, trace_vzac], layout=layout_vel_des)

# --- Save HTML files ---
position_path = os.path.join(BASE_DIR, "position_plot.html")
velocity_path = os.path.join(BASE_DIR, "velocity_plot.html")
command_path = os.path.join(BASE_DIR, "command_plot.html")
pos_des_path = os.path.join(BASE_DIR, "pos_des_plot.html")
vel_des_path = os.path.join(BASE_DIR, "vel_des_plot.html")

fig_pos.write_html(position_path)
fig_vel.write_html(velocity_path)
fig_cmd.write_html(command_path)
fig_pos_des.write_html(pos_des_path)
fig_vel_des.write_html(vel_des_path)

print("Plots saved to:")
print(f"  {position_path}")
print(f"  {velocity_path}")
print(f"  {command_path}")
print(f"  {pos_des_path}")
print(f"  {vel_des_path}")

# --- Try to open the files in Windows Chrome (if WSL) ---
try:
    subprocess.Popen(["/mnt/c/Program Files/Google/Chrome/Application/chrome.exe", position_path])
    subprocess.Popen(["/mnt/c/Program Files/Google/Chrome/Application/chrome.exe", velocity_path])
    subprocess.Popen(["/mnt/c/Program Files/Google/Chrome/Application/chrome.exe", command_path])
    subprocess.Popen(["/mnt/c/Program Files/Google/Chrome/Application/chrome.exe", pos_des_path])
    subprocess.Popen(["/mnt/c/Program Files/Google/Chrome/Application/chrome.exe", vel_des_path])
except FileNotFoundError:
    print("Couldn't open in Chrome. Please open the HTML files manually.")
