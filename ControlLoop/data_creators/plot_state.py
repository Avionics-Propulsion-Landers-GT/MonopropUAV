import pandas as pd
import plotly.graph_objects as go
import subprocess
import os

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

# Open the HTML file in the default Windows browser using WSL
subprocess.run(["chrome.exe", win_path])  # For Ubuntu 20.04+, use `wslview`
