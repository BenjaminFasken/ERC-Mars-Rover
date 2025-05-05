#!/bin/bash

# Check if navigation package is built, if not, build the workspace
if [ ! -d "install/navigation" ]; then
    echo "Navigation package not found. Building the workspace..."
    # Navigate to the workspace root and build the navigation package
    colcon build --packages-select navigation
    if [ $? -ne 0 ]; then
        echo "Failed to build the navigation package. Exiting."
        exit 1
    fi
fi

# Source the ROS 2 workspace (done early to ensure ROS 2 environment is ready)
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start the ROS 2 command in the background, suppressing its output
ros2 run navigation cmd_repeater.py > /dev/null 2>&1 &

# Store the ROS 2 process ID
ROS_PID=$!

# Navigate to the docker directory and start docker compose in the foreground
cd docker
docker compose up

# Store the exit status of docker compose
DOCKER_EXIT_STATUS=$?

# Return to the workspace root
cd ..

# Check if docker compose exited successfully
if [ $DOCKER_EXIT_STATUS -ne 0 ]; then
    echo "Docker Compose failed with exit status $DOCKER_EXIT_STATUS. Exiting."
    # Optionally kill the ROS 2 process if Docker fails
    kill $ROS_PID
    exit $DOCKER_EXIT_STATUS
fi

# Wait for the ROS 2 process to complete
wait $ROS_PID