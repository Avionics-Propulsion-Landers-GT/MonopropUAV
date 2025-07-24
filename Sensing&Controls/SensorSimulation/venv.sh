#!/usr/bin/env bash
set -euo pipefail

# 1) Ensure python3-venv is installed, then create a venv/ folder if it doesn't already exist
if ! python3 -m venv --help > /dev/null 2>&1; then
  echo "python3-venv is not installed. Installing..."
  sudo apt-get update && sudo apt-get install -y python3-venv
fi

if [[ ! -d "venv" ]]; then
  echo "Creating virtualenv in ./venv"
  if ! python3 -m venv venv; then
    echo "Error: Failed to create virtual environment. Check permissions and dependencies."
    exit 1
  fi
fi

# Ensure the venv was created successfully before activating
if [[ -f "venv/bin/activate" ]]; then
  # 2) Activate it
  #    (this only affects this script's shell, not your parent shell)
  source venv/bin/activate
else
  echo "Error: venv/bin/activate not found. Virtual environment creation failed."
  echo "Current directory: $(pwd)"
  echo "Directory contents:"
  ls -l
  exit 1
fi

# 3) Upgrade pip inside the venv
echo "Upgrading pip…"
python -m pip install --upgrade pip

# 4) Install your packages
echo "Installing packages"
python -m pip install numpy

echo "Done. To start using it, run:"
echo "  source venv/bin/activate"
echo "  python your_script.py"
