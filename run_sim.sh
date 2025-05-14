#!/bin/bash

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

# Check and build perception image if necessary
SIM_IMAGE="erc-mars-rover_sim_isaac:latest"
if ! check_docker_image "$SIM_IMAGE"; then
    echo "Building Isaac image $SIM_IMAGE..."
    # Ensure base image exists
    build_base_image
    docker build -t "$SIM_IMAGE" -f docker/Dockerfile_sim_isaac .
    if [ $? -ne 0 ]; then
        echo "Failed to build isaac image $SIM_IMAGE. Exiting."
        exit 1
    fi
fi

# Check and build navigation image if necessary
NAV_IMAGE="erc-mars-rover_sim_nav:latest"
if ! check_docker_image "$NAV_IMAGE"; then
    echo "Building navigation image $NAV_IMAGE..."
    # Ensure base image exists
    build_base_image
    docker build -t "$NAV_IMAGE" -f docker/Dockerfile_sim_nav .
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
docker compose -f docker-compose_sim.yaml up

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
