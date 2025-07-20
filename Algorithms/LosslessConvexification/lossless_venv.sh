#!/usr/bin/env bash
set -euo pipefail

# 1) Create a venv/ folder if it doesn't already exist
if [[ ! -d "venv" ]]; then
  echo "Creating virtualenv in ./venv"
  python3 -m venv venv
fi

# 2) Activate it
#    (this only affects this script's shell, not your parent shell)
source venv/bin/activate

# 3) Upgrade pip inside the venv
echo "Upgrading pip…"
python -m pip install --upgrade pip

# 4) Install your packages
echo "Installing numpy, cvxpy…"
python -m pip install numpy cvxpy ecos scs

echo "Done. To start using it, run:"
echo "  source venv/bin/activate"
echo "  python your_landing_script.py"
