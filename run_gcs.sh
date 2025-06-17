#!/bin/bash

# Verify that the script is being run from the workspace root
if [ ! -d "install" ]; then
    echo "Error: This script must be run from the workspace root (missing 'install' directory)."
    exit 1
fi

# Define RViz2 config file
RVIZ_CONFIG="$(pwd)/src/gcs/rviz2/rviz-config.rviz"

# Source the ROS 2 workspace
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS 2 Humble setup file not found at /opt/ros/humble/setup.bash."
    exit 1
fi

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "Error: Workspace setup file not found at install/setup.bash."
    exit 1
fi

# Run RViz2 with config file if it exists
echo "Running RViz2..."
if [ -f "${RVIZ_CONFIG}" ]; then
    ros2 run rviz2 rviz2 -d "${RVIZ_CONFIG}" &
else
    echo "Warning: RViz config file ${RVIZ_CONFIG} not found, launching RViz with default settings."
    ros2 run rviz2 rviz2 &
fi
RVIZ_PID=$!
sleep 5
if ! ps -p $RVIZ_PID > /dev/null; then
    echo "Error: RViz failed to start."
    exit 1
fi
echo "RViz is running with PID ${RVIZ_PID}."

# Wait for RViz to exit
wait $RVIZ_PID