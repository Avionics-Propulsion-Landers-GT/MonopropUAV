import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

#!/usr/bin/env python3
"""
plot_history.py

A Python script to read a histories.csv file containing the history of 
the position of an aerial vehicle over a series of iterations, and to plot 
the position(s) against the iteration step.

Usage:
    python plot_history.py histories.csv

If the CSV file contains multiple columns (for example, positions along different axes),
each column is plotted as its own line in the graph.
"""

import pandas as pd
import matplotlib.pyplot as plt
import sys

def plot_history(csv_file):
    # Try to read the CSV file
    try:
        # If your CSV file does not have a header row, you can use header=None:
        # data = pd.read_csv(csv_file, header=None)
        # data.columns = ['Position']  # Optionally rename the single column
        data = pd.read_csv(csv_file, header=None)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        sys.exit(1)
    
    # Check if data is empty
    if data.empty:
        print("The provided CSV file is empty!")
        sys.exit(1)
    
    # Create an array of iteration steps from the index
    iterations = np.array(data.index)

    # Create a new figure for the plot
    plt.figure(figsize=(10, 6))
    
    # If there's more than one column, we'll plot each column separately.
    # Each column can represent a different dimension of position (e.g., x, y, z)
    for col in data.columns:
        yVals = np.asarray(data[col])
        # 0 - x, 1 - y, 2 - z
        plt.plot(iterations, yVals, marker='o', linestyle='-', label=col)
    
    # Adding labels, title, grid, and legend for clarity
    plt.xlabel("Iteration Step")
    plt.ylabel("Position")
    plt.title("Aerial Vehicle Position vs. Iteration Step")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    
    # Display the plot
    plt.show()

def main():
    # Check for command-line arguments
    if len(sys.argv) < 2:
        print("Usage: python plot_history.py histories.csv")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    plot_history(csv_file)

if __name__ == "__main__":
    main()
