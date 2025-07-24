#!/bin/bash

set -e  # Exit immediately on error

echo "🔧 Navigating to build directory..."
cd IntegratedDynModel/build/

echo "⚙️ Running cmake configuration..."
cmake .. > /dev/null

echo "🔨 Building the project..."
cmake --build . > /dev/null

echo "🚀 Running the simulation executable..."
./monoprop_sim

echo "📈 Running the WSL Python plotting script..."
cd ..
python3 wslIntegratedPlotter.py

# OR: If you want to use the other plotter:

# echo "📈 Running the Integrated Python plotting script..."
# cd ..
# python3 IntegratedPlotter.py

echo "✅ Done! Simulation and plotting complete."

