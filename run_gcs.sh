#!/bin/bash

echo "=== Checking dependencies ==="

# Check for ROS package
if ! dpkg -l | grep -q ros-humble-rosbridge-server; then
  echo "Installing ros-humble-rosbridge-server..."
  sudo apt-get update
  sudo apt-get install -y ros-humble-rosbridge-server
else
  echo "ros-humble-rosbridge-server already installed"
fi

# Check for Python packages
missing_py_packages=()
for package in pywebview python-nmap requests; do
  if ! pip list | grep -q "^$package "; then
    missing_py_packages+=("$package")
  fi
done

if [ ${#missing_py_packages[@]} -gt 0 ]; then
  echo "Installing missing Python packages: ${missing_py_packages[*]}"
  pip install "${missing_py_packages[@]}"
else
  echo "All required Python packages already installed"
fi

echo "=== Building workspace ==="
# Navigate to workspace root (assuming script is in workspace root)
cd "$(dirname "$0")"
colcon build --packages-select interfaces gcs

echo "=== Setting up ROS environment ==="
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== Launching GCS ==="
echo "Note: Make sure both you and the rover are connected to the same network and subnet."
ros2 launch gcs gcs.launch.py