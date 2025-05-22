# MARS - Leorover ROS2 Workspace

Welcome to the **MARS (Mobile Autonomous Rover System)** ROS2 workspace, designed for the Leorover system. This repository contains a complete system comprising three core components:

- **Vision System**: Processes camera input using a ZED camera.
- **Navigation System**: Manages autonomous navigation and path planning.
- **Mapping System**: Performs SLAM (Simultaneous Localization and Mapping).

These components are containerized into three Docker images:

- **Camera Container**: Interfaces with the ZED camera for image capture.
- **Perception Container**: Runs YOLO for real-time object detection.
- **Navigation & SLAM Container**: Handles navigation and mapping tasks.

This README provides instructions for setting up, building, and running the system on a compatible device.

## Running the Configured System

If your system is already configured, you can skip the setup steps and follow these instructions to start the rover.
1. **Turn on router**:
   - Make sure the SpaceRobticsMobile router is powered on and within reach.
   - Connect your pc to it. Password is: spacerobotics

2. **Power On the System**:
   - Insert one or two fully charged batteries.
   - Ensure both power switches are enabled.

3. **SSH into the System**:
   ```bash
   ssh leorover@192.168.10.22
   ```

4. **Navigate to the Repository**:
   ```bash
   cd ~/ERC-Mars-Rover
   ```

### Starting the Rover

The system includes a bash script, `run_leorover.sh`, which automates the setup and launch process. It verifies that the required Docker images are built (building them if necessary) and starts the system using Docker Compose.

Run the script:
```bash
./run_leorover.sh
```

If the system is already configured, this script will launch the Docker Compose setup directly.

### Launching the Ground Control Station (GCS)
Benajmin

## Software Prerequisites

To set up the MARS system from scratch, ensure your device meets the following requirements:

- **Device**: NVIDIA Jetpack 6.2 compatible device with CUDA developer tools installed.
- **Storage**: At least 50 GB of free disk space for Docker images.
- **Software**:
  - Docker
  - Docker Compose
  - Git

### Step 1: Install Docker

1. Install Docker using the official convenience script:
   ```bash
   curl -fsSL https://get.docker.com -o get-docker.sh
   sudo sh get-docker.sh
   ```

2. **Post-Installation**:
   To run Docker without `sudo`, add your user to the `docker` group:
   ```bash
   sudo groupadd docker
   sudo usermod -aG docker $USER
   newgrp docker
   ```
3. Install Docker Compose to manage multi-container setups:
  ```bash
  sudo apt-get update
  sudo apt-get install -y docker-compose-plugin
  ```
### Step 2: Cloning the Repository

Clone the repository to your home directory:
```bash
cd ~
git clone git@github.com:BenjaminFasken/ERC-Mars-Rover.git
```
### Step 3: Setup LiDAR

When setting up the LiDAR, two different scenarios are possible.<br>
1. Directly connecting the LiDAR to the Ethernet port of the PC (This is the case of Leorover_2).
2. Connecting the LiDAR through a switch (This is the case on Leorover_1).

**Note:** Every time you configure the static IP of the Ethernet port on the PC, remember to reboot the network driver, easily done by disabling the Ethernet port and enabling it again.

**Scenario 1**<br>
1.  Configure the IP of the connected Ethernet port on the PC to the following static IP: `192.168.1.50`, with the submask: `255.255.255.0`
   > **Note:** Ensure that the wifi does not use the same subnet ID as the static IP (`192.168."1".50`), if it does, configure either the router, or the static IP to an unique subnet.
2. Modify the `MID360_config.json` file:
    - Set `cmd_data_ip`, `push_msg_ip`, `point_data_ip`, and `imu_data_ip` to the static ip configured above: `192.168.1.50`
    - Set `lidar_config_ip` to: `192.168.1.1xx` where the last two digits are the last two digits of the LiDAR' serial number.
   
**Scenario 2**<br>
1. Modify the `MID360_config.json` file:
    - Set `cmd_data_ip`, `push_msg_ip`, `point_data_ip`, and `imu_data_ip` to the IP of the host. In Leorover_1's case when connected to Spacerobotics_Mobile: `192.168.10.22`.
    - Set `lidar_config_ip` to: `192.168.10.1xx` where the last two digits are the last two digits of the LiDAR' serial number.
   > **Note:** Make sure that the Ethernet port on the PC is not configured to an static IP.

**If the IP's does not work**
- Check the LiDAR's IP's by connecting it to Livox Viewer found here [Livox Viewer](https://www.livoxtech.com/mid-360/downloads), and check the settings to see if the IP's are correct, if not correct them and reboot the LiDAR



For more help, look at the Livox ROS2 Driver documentation: [livox_ros_driver2 documentation](https://github.com/Livox-SDK/livox_ros_driver2)


### Step 4: Building Docker Images

The system uses three Docker images built from a base image. You can either build the images yourself or load pre-built images from a USB or another system.

#### Option 1: Let the convenience script build the images for you
Navigate to the repository directory:
```bash
cd ~/ERC-Mars-Rover
```

Run the script:
```bash
./run_leorover.sh
```

#### Option 2: Build the Images (optional)

Navigate to the repository directory:
```bash
cd ~/ERC-Mars-Rover
```

Build the images in the following order:

1. **Base Image**:
   ```bash
   docker build -t erc-mars-rover_base:latest -f docker/Dockerfile_base .
   ```

2. **Perception Image**:
   ```bash
   docker build -t erc-mars-rover_perception:latest -f docker/Dockerfile_perception .
   ```

3. **Navigation Image**:
   ```bash
   docker build -t erc-mars-rover_nav:latest -f docker/Dockerfile_nav .
   ```

4. **Isaac Simulation Images (Optional)**:
   a. **Isaac & Perception Simulation Image**:
      ```bash
      docker build -t erc-mars-rover_sim_isaac:latest -f docker/Dockerfile_sim_isaac .
      ```
   b. **Navigation & Mapping Image**:
      ```bash
      docker build -t erc-mars-rover_sim_nav:latest -f docker/Dockerfile_sim_nav .
      ```

> **Note**: Building images requires significant disk space (~50 GB) and time. Ensure sufficient resources are available.

#### Option 3: Transfer Pre-Built Images (optional)

If pre-built images are available, transfer and load them instead of building from scratch.

1.  Save Images (On the Source System)
Save the images to files:
```bash
docker save -o erc-mars-rover_perception.tar erc-mars-rover_perception:latest
docker save -o erc-mars-rover_nav.tar erc-mars-rover_nav:latest
```

2. Transfer Images
Transfer the files to the target system using SCP or a USB drive. Example using SCP:
```bash
scp erc-mars-rover_perception.tar leorover@192.168.10.22:/home/leorover/
```

3.  Load Images
On the target system, load the images:
```bash
docker load -i erc-mars-rover_perception.tar
docker load -i erc-mars-rover_nav.tar
```

#### Running Individual Containers (Optional)

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

- **Stop a Container**:
  From inside the container, type `exit`, or from outside:
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
  nmap -sP 192.168.10.1/24
  ```

- **Launch ZED Wrapper Manually** (inside a container):
  ```bash
  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
  ```

- **Trigger Exploration**:
  ```bash
  ros2 topic pub --once /trigger_exploration std_msgs/Bool "data: true"
  ```

## Troubleshooting

- **Docker Permission Issues**: Ensure your user is in the `docker` group (`groups $USER`). If not, re-run the post-installation steps.
- **Network Connectivity**: Verify the rover’s IP address and network configuration if SSH or GCS fails to connect.
- **Disk Space**: If builds fail, check available disk space with `df -h`.

## Contributing

Contributions are welcome! Please submit pull requests or open issues on the [GitHub repository](git@github.com:BenjaminFasken/ERC-Mars-Rover.git).
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
   docker build -t erc-mars-rover_base:latest -f docker/Dockerfile_base .
   ```

2. **Perception Image**:
   ```bash
   docker build -t erc-mars-rover_perception:latest -f docker/Dockerfile_perception .
   ```

3. **Navigation Image**:
   ```bash
   docker build -t erc-mars-rover_nav:latest -f docker/Dockerfile_nav .
   ```

4. **Isaac Simulation Images (Optional)**:
   a. **Isaac & Perception Simulation Image**:
      ```bash
      docker build -t erc-mars-rover_sim_isaac:latest -f docker/Dockerfile_sim_isaac .
      ```
   b. **Navigation & Mapping Image**:
      ```bash
      docker build -t erc-mars-rover_sim_nav:latest -f docker/Dockerfile_sim_nav .
      ```

> **Note**: Building images requires significant disk space (~50 GB) and time. Ensure sufficient resources are available.

### Option 2: Transfer Pre-Built Images

If pre-built images are available, transfer and load them instead of building from scratch.

#### Save Images (On the Source System)
Save the images to files:
```bash
docker save -o erc-mars-rover_perception.tar erc-mars-rover_perception:latest
docker save -o erc-mars-rover_nav.tar erc-mars-rover_nav:latest
```

#### Transfer Images
Transfer the files to the target system using SCP or a USB drive. Example using SCP:
```bash
scp erc-mars-rover_perception.tar leorover@192.168.10.22:/home/leorover/
```

#### Load Images
On the target system, load the images:
```bash
docker load -i erc-mars-rover_perception.tar
docker load -i erc-mars-rover_nav.tar
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

- **Stop a Container**:
  From inside the container, type `exit`, or from outside:
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
  nmap -sP 192.168.10.1/24
  ```

- **Launch ZED Wrapper Manually** (inside a container):
  ```bash
  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
  ```

- **Trigger Exploration**:
  ```bash
  ros2 topic pub --once /trigger_exploration std_msgs/Bool "data: true"
  ```

## Troubleshooting

- **Docker Permission Issues**: Ensure your user is in the `docker` group (`groups $USER`). If not, re-run the post-installation steps.
- **Network Connectivity**: Verify the rover’s IP address and network configuration if SSH or GCS fails to connect.
- **Disk Space**: If builds fail, check available disk space with `df -h`.

## Contributing

Contributions are welcome! Please submit pull requests or open issues on the [GitHub repository](git@github.com:BenjaminFasken/ERC-Mars-Rover.git).
```
