import pandas as pd
import plotly.graph_objs as go
import plotly.offline as pyo
import os
import subprocess

# Get base directory
BASE_DIR = os.path.abspath(os.path.dirname(__file__))

# Load the simulation data
csv_path = os.path.join(BASE_DIR, "build/simulation_results.csv")
data = pd.read_csv(csv_path)

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

# --- Save HTML files ---
position_path = os.path.join(BASE_DIR, "position_plot.html")
velocity_path = os.path.join(BASE_DIR, "velocity_plot.html")

pyo.plot(fig_pos, filename=position_path, auto_open=False)
pyo.plot(fig_vel, filename=velocity_path, auto_open=False)

print("Plots saved to:")
print(f"  {position_path}")
print(f"  {velocity_path}")

# --- Try to open the files in Windows browser (if WSL) ---
try:
    subprocess.Popen(["/mnt/c/Program Files/Google/Chrome/Application/chrome.exe", position_path])
    subprocess.Popen(["/mnt/c/Program Files/Google/Chrome/Application/chrome.exe", velocity_path])
except FileNotFoundError:
    print("Couldn't open in Chrome. Please open the HTML files manually.")
