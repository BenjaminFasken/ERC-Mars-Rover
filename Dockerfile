# Use Ubuntu 22.04 ARM64 as the base image (for JetPack 6)
FROM arm64v8/ubuntu:22.04

# Set up timezone non-interactively
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y tzdata && \
    ln -fs /usr/share/zoneinfo/UTC /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata

# Install ROS 2 Humble and basic tools
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    git \
    wget \
    bash \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && apt-get update && apt-get install -y \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install minimal dependencies for ZED ROS wrapper (no SDK install)
RUN apt-get update && apt-get install -y \
    libusb-1.0-0-dev \
    libgl1-mesa-glx \
    libegl1-mesa \
    libx11-dev \
    libxau-dev \
    libxdmcp-dev \
    libxcb1-dev \
    libglu1-mesa-dev \
    ros-humble-vision-opencv \
    ros-humble-zed-interfaces \
    && pip3 install ultralytics \
    && rm -rf /var/lib/apt/lists/*

# Set up the ERC-Mars-Rover workspace
WORKDIR /ERC-Mars-Rover

# Copy the existing src/ directory from the host (navigation, probe_detection, slam)
COPY src/ /ERC-Mars-Rover/src/

# Clone zed-ros2-wrapper into the src/ directory
RUN cd /ERC-Mars-Rover/src && \
    git clone https://github.com/stereolabs/zed-ros2-wrapper.git

# Install dependencies from all packages in src/
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release"

# Source the workspace setup in the container
ENTRYPOINT ["/bin/bash", "-c", "source /ERC-Mars-Rover/install/setup.bash && exec \"$@\"", "bash"]