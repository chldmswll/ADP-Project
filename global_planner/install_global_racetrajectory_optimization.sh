#!/bin/bash
# Script to install global_racetrajectory_optimization Python module
# This script should be run from the planner directory or race_stack workspace

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_DIR="${SCRIPT_DIR}/python_modules/global_racetrajectory_optimization"

echo "Installing global_racetrajectory_optimization module..."
echo "Module directory: ${MODULE_DIR}"

if [ ! -f "${MODULE_DIR}/setup.py" ]; then
    echo "Error: setup.py not found at ${MODULE_DIR}/setup.py"
    exit 1
fi

# Check if we're in a virtual environment or should use user install
if [ -n "$VIRTUAL_ENV" ]; then
    echo "Installing in virtual environment: $VIRTUAL_ENV"
    pip install -e "${MODULE_DIR}"
elif [ -n "$ROS_DISTRO" ]; then
    echo "ROS environment detected. Installing in user space..."
    pip install --user -e "${MODULE_DIR}"
else
    echo "Installing in user space..."
    pip install --user -e "${MODULE_DIR}"
fi

# Also add to PYTHONPATH for immediate use
PYTHON_MODULES_PATH="${SCRIPT_DIR}/python_modules"
if [[ ":$PYTHONPATH:" != *":${PYTHON_MODULES_PATH}:"* ]]; then
    echo ""
    echo "To make the module available, add this to your ~/.bashrc or ~/.zshrc:"
    echo "export PYTHONPATH=\${PYTHONPATH}:${PYTHON_MODULES_PATH}"
    echo ""
    echo "Or run this command now:"
    echo "export PYTHONPATH=\${PYTHONPATH}:${PYTHON_MODULES_PATH}"
fi

echo "Installation complete!"
echo ""
echo "To verify installation, run:"
echo "python3 -c 'from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer; print(\"Success!\")'"

