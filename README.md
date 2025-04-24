# MARS - Leorover ROS2 Workspace

Welcome to the MARS (Mobile Autonomous Rover System) ROS2 workspace, designed for the Leorover system. This repository contains a complete system comprising three core components:

- **Vision System**: Handles camera input using a ZED camera.
- **Navigation System**: Manages autonomous navigation.
- **Mapping System**: Performs SLAM (Simultaneous Localization and Mapping).

These components are containerized into three Docker images:

- **Camera Container**: Interfaces with the ZED camera.
- **Perception Container**: Runs YOLO for object detection.
- **Navigation & SLAM Container**: Handles navigation and mapping.

This README guides you through setting up, building, and running the system on a compatible device.

## Software prerequisites

To run the MARS system, ensure your setup meets the following requirements:

- **Device**: NVIDIA Jetpack 6.2 compatible device with CUDA developer tools installed.
- **Storage**: At least 50 GB of free disk space for Docker images.
- **Software**:
  - Docker
  - Docker Compose
  - Git

### Step 1: Install Docker

1. Install Docker using the convenience script:

   ```bash
   curl -fsSL https://get.docker.com -o get-docker.sh
   sudo sh get-docker.sh
   ```

2. **Post-Installation**:
   To run Docker without `sudo`, add your user to the `docker` group and configure permissions:

   ```bash
   sudo groupadd docker
   sudo usermod -aG docker $USER
   newgrp docker
   ```
### Step 2: Install Docker Compose

Install Docker Compose to manage multi-container setups:

```bash
sudo apt-get update
sudo apt-get install -y docker-compose-plugin
```

## Cloning the Repository

Clone the repository to your home directory:

```bash
cd ~
git clone git@github.com:BenjaminFasken/ERC-Mars-Rover.git
```

## Building Docker Images

The system uses three Docker images built from a base image. You can either build the images yourself or load pre-built images from a USB or another system.

### Option 1: Build the Images

Navigate to the repository directory:

```bash
cd ~/ERC-Mars-Rover
```

Build the images in the following order:

1. **Base Image**:

   ```bash
   docker build -t erc-mars-rover_base:latest -f Dockerfile_base .
   ```

2. **Perception Image**:

   ```bash
   docker build -t erc-mars-rover_perception:latest -f Dockerfile_perception .
   ```

3. **Navigation Image**:

   ```bash
   docker build -t erc-mars-rover_nav:latest -f Dockerfile_nav .
   ```

4. **Simulation Image (Optional)** 
  ```bash
   docker build -t erc-mars-rover_sim:latest -f Dockerfile_sim .
  ```

> **Note**: Building the images requires significant disk space and time. Ensure at least 50 GB is available.

### Option 2: Transfer Pre-Built Images (Optional)

If you have pre-built images, you can transfer and load them instead of building from scratch.

#### Save Images (On the Source System)

Save the images to files:

```bash
docker save -o erc-mars-rover_perception.tar erc-mars-rover_perception:latest
docker save -o erc-mars-rover_nav.tar erc-mars-rover_nav:latest
```

#### Transfer Images

Transfer the files to the target system using SCP (over a local network) or a USB drive. Example using SCP:

```bash
scp erc-mars-rover_perception.tar leorover@192.168.0.61:/home/leorover/
```

#### Load Images

On the target system, load the images:

```bash
docker load -i erc-mars-rover_perception.tar
docker load -i erc-mars-rover_nav.tar
```

## Running the System

### Using Docker Compose

To start all containers simultaneously, navigate to the repository directory and run:

```bash
cd ~/ERC-Mars-Rover
docker compose up
```

This launches the entire system with all components.


#### Simulation (Optional)
If you wish to start the simulated system use this command: 

```bash
cd ~/ERC-Mars-Rover
docker-compose -f docker-compose_sim.yml up
```

### Running Individual Containers (Optional)

To test individual containers, use the following commands. These require privileged access and GPU support:

1. **ZED Camera Container**:

   ```bash
   sudo docker run --runtime=nvidia -it --name zed_container --privileged --network=host \
   -v /lib/modules/$(uname -r):/lib/modules/$(uname -r) \
   -v /run/nvargus:/run/nvargus \
   -v /tmp/argus_socket:/tmp/argus_socket \
   --device=/dev/video0 \
   --entrypoint /bin/bash erc-mars-rover_perception:latest
   ```

2. **Perception Container**:

   ```bash
   sudo docker run --runtime=nvidia -it --name perception_container --privileged --network=host \
   -v /lib/modules/$(uname -r):/lib/modules/$(uname -r) \
   -v /run/nvargus:/run/nvargus \
   -v /tmp/argus_socket:/tmp/argus_socket \
   --device=/dev/video0 \
   --entrypoint /bin/bash erc-mars-rover_perception:latest
   ```

3. **Navigation Container**:

   ```bash
   sudo docker run --runtime=nvidia -it --name nav_container --privileged --network=host \
   -v /lib/modules/$(uname -r):/lib/modules/$(uname -r) \
   -v /run/nvargus:/run/nvargus \
   -v /tmp/argus_socket:/tmp/argus_socket \
   --device=/dev/video0 \
   --entrypoint /bin/bash erc-mars-rover_nav:latest
   ```

#### Managing Individual Containers

- **Stop a Container**: From inside the container, type `exit`, or from outside:

  ```bash
  docker stop <container_name>
  ```

- **Restart a Container**:

  ```bash
  docker start <container_name>
  docker attach <container_name>
  ```

  Example:

  ```bash
  docker start perception_container
  docker attach perception_container
  ```

## Useful Commands

- **Check Connected Devices on the Network**:

  ```bash
  nmap -sP 192.168.0.1/23
  ```

- **Launch ZED Wrapper Manually** (inside a container):

  ```bash
  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
  ```
