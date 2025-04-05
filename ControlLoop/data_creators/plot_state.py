import pandas as pd
import plotly.graph_objects as go
import subprocess
import os

"""

    plot_state.py

    by spencer boebel

    PURPOSE: Plot a Plotly graph of state.csv and display it
    as a HTML file in the browser. FOR WINDOWS USERS: you will have 
    to rewrite the bottom half of the code to make this work for you,
    or open the project in WSL (recommended)

"""

# Load the CSV file
file_path = "../state.csv"  # Make sure the file is in the same directory
df = pd.read_csv(file_path)

# Create an interactive plot with Plotly
fig = go.Figure()

# Add traces for X, Y, and Z
fig.add_trace(go.Scatter(x=df["Iteration"], y=df["x"], mode='lines', name='X'))
fig.add_trace(go.Scatter(x=df["Iteration"], y=df["y"], mode='lines', name='Y'))
fig.add_trace(go.Scatter(x=df["Iteration"], y=df["z"], mode='lines', name='Z'))

# Update layout
fig.update_layout(
    title="Interactive Plot of X, Y, and Z vs. Iteration",
    xaxis_title="Iteration",
    yaxis_title="Values",
    legend_title="State Variables",
    template="plotly_dark",
)

# Save the figure as an HTML file
html_file = "plot.html"
fig.write_html(html_file)

wsl_html_path = os.path.abspath("plot.html")
fig.write_html(wsl_html_path)


distro_name = "Ubuntu"
win_path = f"\\\\wsl.localhost\\{distro_name}" + wsl_html_path.replace("/", "\\")

fig2 = go.Figure()
fig2.add_trace(go.Scatter(x=df["Iteration"], y=df["theta_x"], mode='lines', name='Theta X'))
fig2.add_trace(go.Scatter(x=df["Iteration"], y=df["theta_y"], mode='lines', name='Theta Y'))
fig2.add_trace(go.Scatter(x=df["Iteration"], y=df["theta_z"], mode='lines', name='Theta Z'))

fig2.update_layout(
    title="Angular States (Theta X, Y, Z) vs. Iteration",
    xaxis_title="Iteration",
    yaxis_title="Angle (radians or degrees)",
    legend_title="Angular States",
    template="plotly_dark",
)

fig2_html_file = "plot_angular.html"
fig2.write_html(fig2_html_file)

wsl_html_path2 = os.path.abspath("plot_anglular.html")
fig2.write_html(wsl_html_path2)

win_path2 = f"\\\\wsl.localhost\\{distro_name}" + wsl_html_path2.replace("/", "\\")

fig3 = go.Figure()
fig3.add_trace(go.Scatter(x=df["Iteration"], y=df["vx"], mode='lines', name='VX'))
fig3.add_trace(go.Scatter(x=df["Iteration"], y=df["vy"], mode='lines', name='VY'))
fig3.add_trace(go.Scatter(x=df["Iteration"], y=df["vz"], mode='lines', name='VZ'))

fig3.update_layout(
    title="Velocity (X, Y, Z) vs. Iteration",
    xaxis_title="Iteration",
    yaxis_title="Velocity [m/s]",
    legend_title="States",
    template="plotly_dark",
)

fig3_html_file = "plot_velocity.html"
fig3.write_html(fig3_html_file)

wsl_html_path3 = os.path.abspath("plot_velocity.html")
fig3.write_html(wsl_html_path3)

win_path3 = f"\\\\wsl.localhost\\{distro_name}" + wsl_html_path3.replace("/", "\\")

fig4 = go.Figure()
fig4.add_trace(go.Scatter(x=df["Iteration"], y=df["omega_x"], mode='lines', name='Omega X'))
fig4.add_trace(go.Scatter(x=df["Iteration"], y=df["omega_y"], mode='lines', name='Omega Y'))
fig4.add_trace(go.Scatter(x=df["Iteration"], y=df["omega_z"], mode='lines', name='Omega Z'))

fig4.update_layout(
    title="Angular States (Omega X, Y, Z) vs. Iteration",
    xaxis_title="Iteration",
    yaxis_title="Angular Speed (radians or degrees)",
    legend_title="Angular Speed States",
    template="plotly_dark",
)

fig4_html_file = "plot_aacc.html"
fig4.write_html(fig4_html_file)

wsl_html_path4 = os.path.abspath("plot_aacc.html")
fig4.write_html(wsl_html_path4)

win_path4= f"\\\\wsl.localhost\\{distro_name}" + wsl_html_path4.replace("/", "\\")

fig5 = go.Figure()
fig5.add_trace(go.Scatter(x=df["Iteration"], y=df["thrust"], mode='lines', name='Optimal Thrust [N]'))
fig5.add_trace(go.Scatter(x=df["Iteration"], y=df["adot"], mode='lines', name='Optimal A Gimbal [rad]'))
fig5.add_trace(go.Scatter(x=df["Iteration"], y=df["bdot"], mode='lines', name='Optimal B Gimbal [rad]'))

fig5.update_layout(
    title="Desired Command State",
    xaxis_title="Iteration",
    yaxis_title="N / Rad",
    legend_title="Command States",
    template="plotly_dark",
)

fig5_html_file = "plot_cmd.html"
fig5.write_html(fig5_html_file)

wsl_html_path5 = os.path.abspath("plot_cmd.html")
fig5.write_html(wsl_html_path5)

win_path5= f"\\\\wsl.localhost\\{distro_name}" + wsl_html_path5.replace("/", "\\")

# Open the HTML file in the default Windows browser using WSL
subprocess.run(["chrome.exe", win_path])  # For Ubuntu 20.04+, use `wslview`
subprocess.run(["chrome.exe", win_path2])
subprocess.run(["chrome.exe", win_path3])
subprocess.run(["chrome.exe", win_path4])
subprocess.run(["chrome.exe", win_path5])
