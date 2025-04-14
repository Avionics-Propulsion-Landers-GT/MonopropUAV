import pandas as pd
import plotly.graph_objects as go
import subprocess
import os

"""
    wslIntegratedPlotter.py

    by spencer boebel (rewritten)

    PURPOSE: Plot interactive figures using Plotly from `simulation_results.csv`,
    convert WSL paths for Windows, and open them in the browser (via chrome.exe).

    Works best in WSL with a shared browser environment.
"""

# --- Load CSV ---
file_path = "build/simulation_results.csv"
df = pd.read_csv(file_path)

# --- Utility: Write + return win path ---
def save_plot_and_get_windows_path(fig, filename):
    abs_path = os.path.abspath(filename)
    fig.write_html(abs_path)
    distro = "Ubuntu"
    return f"\\\\wsl.localhost\\{distro}" + abs_path.replace("/", "\\")

# --- Plot 1: Position (x, y, z) ---
fig1 = go.Figure()
fig1.add_trace(go.Scatter(x=df["time"], y=df["x"], mode='lines', name='x'))
fig1.add_trace(go.Scatter(x=df["time"], y=df["y"], mode='lines', name='y'))
fig1.add_trace(go.Scatter(x=df["time"], y=df["z"], mode='lines', name='z'))
fig1.update_layout(
    title="Position vs Time",
    xaxis_title="Time",
    yaxis_title="Position",
    legend_title="Axes",
    template="plotly_dark"
)
path1 = save_plot_and_get_windows_path(fig1, "position_plot.html")

# --- Plot 2: Velocity (vx, vy, vz) ---
fig2 = go.Figure()
fig2.add_trace(go.Scatter(x=df["time"], y=df["vx"], mode='lines', name='vx'))
fig2.add_trace(go.Scatter(x=df["time"], y=df["vy"], mode='lines', name='vy'))
fig2.add_trace(go.Scatter(x=df["time"], y=df["vz"], mode='lines', name='vz'))
fig2.update_layout(
    title="Velocity vs Time",
    xaxis_title="Time",
    yaxis_title="Velocity",
    legend_title="Axes",
    template="plotly_dark"
)
path2 = save_plot_and_get_windows_path(fig2, "velocity_plot.html")

# --- Plot 3 (Optional): Add more as needed ---
# Example: Thrust or Command if `thrust`, `a`, `b` are present
if {"thrust", "a", "b"}.issubset(df.columns):
    fig3 = go.Figure()
    fig3.add_trace(go.Scatter(x=df["time"], y=df["thrust"], mode='lines', name='Thrust [N]'))
    fig3.add_trace(go.Scatter(x=df["time"], y=df["a"], mode='lines', name='Gimbal A [rad]'))
    fig3.add_trace(go.Scatter(x=df["time"], y=df["b"], mode='lines', name='Gimbal B [rad]'))
    fig3.update_layout(
        title="Commanded States",
        xaxis_title="Time",
        yaxis_title="Value",
        legend_title="Commands",
        template="plotly_dark"
    )
    path3 = save_plot_and_get_windows_path(fig3, "command_plot.html")
else:
    path3 = None

# --- Open plots in browser (Windows Chrome via WSL) ---
subprocess.run(["chrome.exe", path1])
subprocess.run(["chrome.exe", path2])
if path3:
    subprocess.run(["chrome.exe", path3])
