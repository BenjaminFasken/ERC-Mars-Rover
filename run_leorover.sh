#!/bin/bash

# Check if navigation package is built, if not, build the workspace
if [ ! -d "install/navigation" ]; then
    echo "Navigation package not found. Building the workspace..."
    # Navigate to the workspace root and build the navigation package
    colcon build --packages-select interfaces navigation
    if [ $? -ne 0 ]; then
        echo "Failed to build the navigation package. Exiting."
        exit 1
    fi
fi

# Function to check if a Docker image exists
check_docker_image() {
    local image_name="$1"
    if ! docker image inspect "$image_name" > /dev/null 2>&1; then
        echo "Docker image $image_name not found."
        return 1
    fi
    echo "Docker image $image_name found."
    return 0
}

# Function to build the base image if it doesn't exist
build_base_image() {
    local base_image="erc-mars-rover_base:latest"
    if ! check_docker_image "$base_image"; then
        echo "Building base image $base_image..."
        docker build -t "$base_image" -f docker/Dockerfile_base .
        if [ $? -ne 0 ]; then
            echo "Failed to build base image $base_image. Exiting."
            exit 1
        fi
    fi
}

# Check and build perception image if necessary
PERCEPTION_IMAGE="erc-mars-rover_perception:latest"
if ! check_docker_image "$PERCEPTION_IMAGE"; then
    echo "Building perception image $PERCEPTION_IMAGE..."
    # Ensure base image exists
    build_base_image
    docker build -t "$PERCEPTION_IMAGE" -f docker/Dockerfile_perception .
    if [ $? -ne 0 ]; then
        echo "Failed to build perception image $PERCEPTION_IMAGE. Exiting."
        exit 1
    fi
fi

# Check and build navigation image if necessary
NAV_IMAGE="erc-mars-rover_nav:latest"
if ! check_docker_image "$NAV_IMAGE"; then
    echo "Building navigation image $NAV_IMAGE..."
    # Ensure base image exists
    build_base_image
    docker build -t "$NAV_IMAGE" -f docker/Dockerfile_nav .
    if [ $? -ne 0 ]; then
        echo "Failed to build navigation image $NAV_IMAGE. Exiting."
        exit 1
    fi
fi

# Source the ROS 2 workspace (done early to ensure ROS 2 environment is ready)
source /opt/ros/humble/setup.bash
source install/setup.bash

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
    # Kill the ROS 2 process if Docker fails
    exit $DOCKER_EXIT_STATUS
fi
