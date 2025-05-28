# Simulation Environment for the Leo Rover with Sensors and Mars Terrain

This repository contains a simulation environment developed in **NVIDIA Isaac Sim**, featuring a digital model of the **Leo Rover**—both with and without sensors— in a Mars terrain mad from a 3D scan of a previous version of the ERC Mars Yard. The setup is fully integrated with **ROS2** and supports sensor configurations including:

- Stereolab’s **ZED X** stereo camera  
- **Hesai XT-32** LiDAR  
- **Ouster OS1** 32-channel LiDAR  

The environment allows realistic simulation of a sensor-equipped Leo Rover in Martian-like terrain, with **RTAB-Map** SLAM. All components can be launched from a single launch file for ease of use. The repository includes:

- Launch file for standard Isaac Sim
- Launch files for Isaac Sim with the sensorized Leo Rover in the Martian terrain
- Launch file Isaac Sim with the sensorized Leo Rover in the Martian terrain and the integration with RTAB-Map and the ZED wrapper

To ensure a smooth and reproducible experience, the entire environment is containerized using **Docker**, minimizing dependency issues. The setup is designed to run on a **virtual machine (VM)** using the **Strato HPC service**, available to AAU students.

## 📖 Documentation Overview

1. **Using Strato**  
   Instructions on how to access and use AAU's Strato high-performance computing platform.

2. **Building Docker Images**  
   Guide for creating and managing the necessary Docker containers.

3. **Launching the Full Simulation**  
   How to launch Isaac Sim with the sensorized Leo Rover, ZED wrapper, and RTAB-Map from a unified launch file, and visulize the simulation remotely.

4. **Custom Launch Options**  
   Running specific configurations such as:
   - Isaac Sim (default) alone
   - Isaac Sim with the rover in Martian terrain
   - Other combinations using Python launch files
   - RTAB-Map

5. **VM Visualization**  
   How to visualize data from the VM, use **RViz** with Isaac Sim, and interact with the simulation remotely.

6. **Good to know stuff about Docker and more**

---

> 🛠️ For setup instructions, jump to the [Setup Guide](#)  
> 🚀 To launch the simulation, see the [Usage Instructions](#)


## 1. Using Strato

The **Strato** platform is Aalborg University's high-performance computing (HPC) service, allowing you to run computationally heavy simulations like Isaac Sim in a virtualized environment.

Follow the steps below to access and prepare your Strato environment for use with this simulation setup:

### 1.1 Setup instance/VM on Strato

If you are new to Strato go to `https://hpc.aau.dk/strato/` to get an overview and Introduction.

Compleat Initial Openstack setup (`https://hpc.aau.dk/strato/getting-started/launch-instance/`):
- Make SSH rules
- Create SSH key pair
- Upload key pair to OpenStack

The above you only have to do once. Then you can:
- Launch Ubuntu instance

> **Note**: If you cheched [NO] to **Delete Volume on Instance Delete**, then you can delete your Instance, but your data is saved in the Volume. You can then create a new Instance based on the Volume, and the new Instance will be identical to the one you deleted with the exeption of a new IP-adress and "reboot".


### 1.1 Connecting to Your Strato Instance

To connect to your Strato instance, use the following SSH command in the terminal (on your local PC). Replace `<my_private_key>` with the name of your SSH key file, and replace `10.92.1.zzz` with the actual IP address of your Strato instance:

```bash
ssh -X -i ~/.ssh/<my_private_key> ubuntu@10.92.1.zzz
```

- Type `yes` to leave fingerprint
- Enter the passphrase (pasword) for your SSH key when asked.

Once connected, you should now see the terminal prompt of the Strato VM.

### 1.2 Setup GitHub on your VM
If you have not allrady then now is the time to set up GitHub on your VM.

Clone the repository to your home directory:
```bash
cd ~
git clone git@github.com:BenjaminFasken/ERC-Mars-Rover.git
```

Check if your are in the `develoment` branch on the VM, by one of the two following comands:
```bash
cd ~/ERC-MARS-ROVER
git branch
```

Alternatively:
```bash
cd ~/ERC-MARS-ROVER
git rev-parse --abbrev-ref HEAD
```

If not, then swich to the `develoment` branch.

### 1.3 Recomended to set up ROS 2 on VM
It is recommended to set up **ROS 2 Humble** as well.
```bash
`https://docs.ros.org/en/humble/Installation.htm
```

## 2. Building Docker Images
Docker must be installed on your VM. If Docker is already installed, you can proceed to 2.2 Building Docker Images.

### 2.1 Install Docker

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

### 2.2 Building Docker Images


You have two options for building the required Docker images. **Option 1** is the recommended approach as it automates the full setup and launches everthing.

**Option 1 will:**
1. Builds the Isaac & Perception Simulation Image and the Navigation & Mapping Image automaticly
2. Launches Isaac Sim
   - With the digital model equiped with ZEDx Stereo Camera and Hesai XT-32 LiDAR in the the ERC Mars Yard
3. Launches the ZED wrapper
4. Launches RTAB-Map

Optionaly the two Docker images can be build induvidauly, using **option 2**.

Alternatively, you can build the two Docker images individually using **option 2**, and launch components manually (or automaticly using `./run_sim.sh`). This approach is helpful if Option 1 fails or if you prefer finer control over the setup process.


#### **Option 1**: Let the convenience script build the images for you
Navigate to the repository directory:
```bash
cd ~/ERC-Mars-Rover
```

Run the script:
```bash
./run_sim.sh
```

#### **Option 2**: Build the Images (optional)

Navigate to the repository directory:
```bash
cd ~/ERC-Mars-Rover
```

Build the images in the following order:

1. **Isaac & Perception Simulation Image (Optional)**:
   ```bash
   docker build -t erc-mars-rover_sim_isaac:latest -f docker/Dockerfile_sim_isaac .
      ```
2. **Navigation & Mapping Image (Optional)**:
   ```bash
   docker build -t erc-mars-rover_sim_nav:latest -f docker/Dockerfile_sim_nav .
   ```

> **Note**: Building images requires significant disk space (~50 GB) and time. Ensure sufficient resources are available.


## 3. Launching the Full Simulation
To launch Isaac Sim with the sensorized Leo Rover, ZED wrapper, and RTAB-Map

Navigate to the repository directory (VM):
```bash
cd ~/ERC-Mars-Rover
```

Run the script (VM):
```bash
./run_sim.sh
```

To see the simulation on your local PC you need the **Isaac Sim WebRTC Streaming Client**.
a. **If not downloaded** go to `https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html#isaac-sim-latest-release` and Download the Isaac Sim WebRTC Streaming Client for linux, and put it in a folder called `/isaacsim` on your local machine home directory.

b. **If downloaded**, then in your local machine home directory run:
   ```bash
   ./isaacsim/isaacsim-webrtc-streaming-client-1.0.6-linux-x64.AppImage
   ```
   - Enter your VM's IP adress in **Server: **`10.92.1.zzz`
   - Chose your **Resolution:** `1920 x 1080 (FHD)`
   - Then **Cpnnect** or hit `enter`

   - To reload: `F5`
   - For Full Screen: `F11`


To visualize RTAB-Map output open a terminal on your VM, that allows for visualization (see opint 5. VM Visualization), and run the following to launch **RViz**:
```bash
ros2 run rviz2 rviz2 & sleep 2 && ros2 param set /rviz use_sim_time true
```
- This will launch **RViz**, wait for 2 sec. and then make RViz use simulation time (by subscribing to the ROS2 topic /clock).

## 4. Custom Launch Options

This will include steps to open the two containers individualy. Launching the default Isaac Sim program, launching Isaac Sim with the Leo Rover diffrent variences of the sensor setup in the Mars terrain from made python scripts. Launching RTAB-Map.

This section describes how to manually launch the individual Docker containers and run different components of the simulation environment.

You will see how to:

- Launch the **default Isaac Sim** application.
- Launch **Isaac Sim with the Leo Rover**, using different sensor configurations in the **ERC Mars Yard**, via custom Python launch scripts.
- Launch the **ZED wrapper** for Isaac Sim.
- Launch **RTAB-Map** independently for mapping and localization.

These options are useful if you want more control over which components are running, or if you need to debug or test specific configurations separately.

### 4.1 Isaac Sim

Start by launching the Isaac Sim simulation container:
```bash
docker run --name isaac-sim-New --entrypoint bash -it --runtime=nvidia --gpus all \
    -e "ACCEPT_EULA=Y" \
    --network=host \
    -e "PRIVACY_CONSENT=Y" \
    -e DISPLAY=$DISPLAY \
    -e "OMNI_KIT_ALLOW_ROOT=1" \
    -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
    -e "LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib" \
    -e "ROS_DISTRO=humble" \
    -e "NVST_DISABLE_ETLI_LOGS=1" \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/root/Documents:rw \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    erc-mars-rover_sim_isaac:latest
```

Now you have a cubel of options:
 
 a. **Launch the default Isaac Sim application**:
   Navigate to the `/isaac-sim` folder, and run:
   ```bash
   ./isaac-sim.streaming.sh
   ```
   - In the folder `/isaac-sim/assets` you will finde the different variations of the Leo Rover with and without sensors like `LEO_rover_ZED_XT_32.usdz` and `Leo_rover_control.usdz`you will also find a folder `/mars_yard` where you can make the ERC Mars Yard from the `mars_yard.ob` file.

   - If it is the first time your are using the Isaac Sim application make sure to `Enable` the **ROS 2 Bridge** and the **ZED Camera Extention**, by navigating with in the Isaac Sim application to the `Window`tab and select `Extentions`, then search for the **ROS 2 Bridge** and the **ZED Camera Extention**. Make sure they are `Enabled` and set to `ÀUTOLOAD`.
      - This is only relevant if you want to use ROS2 with Isaac Sim, and if want to use ZED Camera with the ZED wrapper or ZED SDK.


b. **Launch Isaac Sim from python script with the Leo Rover and Mars yard**
   Navigate to the `/isaac-sim` folder, and run:
   ```bash
   ./python.sh assets/leorover_xt32_lifted_30fps.py
   ```

   You can also replace `leorover_xt32_lifted_30fps.py` with other Python launch scripts that configure different sensor variations on the Leo Rover in the Mars Yarrd:
   - `leorover_os1_lifted.py`
   - `leorover_xt32_base.py`
   - `leorover_xt32_base.py`
   - `leorover_xt32_lifted.py`


To see the simulaion on your local PC, open a terminal on your local PC and run:
```bash
./isaacsim/isaacsim-webrtc-streaming-client-1.0.6-linux-x64.AppImage
```

To control the Leo Rover velocity comands can be sent on the Ros 2 topic `/cmd_vel`, from a terminal on your VM given ROS 2 Humble is installed.
Comands can be sent directly like:
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{'linear': {'x': 0.2, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.2}}"
```
**or** by Keyboard comands:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 4.2 ZED wrapper for Isaac Sim:
Given that the Isaac Sim simulation container is open and running a simulations which includes the ZEDx camera and ROS2 setup. You can open a new terminal on your VM and run the following comand to enter the simulation container:
```bash
docker exec -it isaac-sim-New bash
```

Now you can run the following to start the ZED wrapper synchronized to the simulation time:
> **Note**: You may need to navigate to the `~/ERC-MARS-ROVER` directory before running the command.
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx sim_mode:=true use_sim_time:=true
```


### 4.3 RTAB-Map for Isaac Sim
Start by launching the mapping container in a new terminal on your VM:
```bash
sudo docker run -it --name nav_container --privileged --network=host \
-v /lib/modules/$(uname -r):/lib/modules/$(uname -r) \
-v /run/nvargus:/run/nvargus \
-v /tmp/argus_socket:/tmp/argus_socket \
--device=/dev/video0 \
--entrypoint /bin/bash erc-mars-rover_sim_nav:latest
```

If not allready, then navigate to the `~/ERC-MARS-ROVER` directory.


You can then launch **RTAB-Map** using the following command, assuming that **Isaac Sim** is already running with the **Leo Rover equipped with both LiDAR and a ZED X camera**, and that the **ZED wrapper** is also active.
```bash
ros2 launch simulation rtab_sim.launch.py
```
## 5. VM Visualization
Setup a secure VNC (Virtual Network Computing) session over SSH to remotely access the graphical desktop environment of your virtual machine (VM). So you can visualize RViz

1. On your VM start a VNC server on display :1, which corresponds to TCP port 5901, by running the following comand in a terminal:
```bash
vncserver :1 -localhost
vncserver -list
```
The comand `vncserver -list` shows a list of active VNC server sessions running on the current machine.shows a list of active VNC server sessions running on the current machine.
- If you need to stop (terminate) the VNC server session running on display :1, then run: `vncserver -kill :1`

2. Initiates an SSH connection to the remote VM using your private key `<my_private_key>`, by running the following in a terminal on your local PC:
```bash
ssh -i ~/.ssh/<my_private_key> -L 5901:localhost:5901 ubuntu@10.92.1.zzz
```
- This creates an SSH tunnel from your local PC's port 5901 to the remote VM's port 5901.
   - `-L 5901:localhost:5901` means: “Forward my local port 5901 to port 5901 on the VM’s localhost interface.”

- Make sure to replace  `<my_private_key>` with the name of your SSH key file, and replace `10.92.1.zzz` with the actual IP address of your Strato instance.

3. Launches your VNC client (viewer), we have used the program **RealVNC Viewer**, and wich launches in a terminal on your local PC:
```bash
vncviewer
```

Then in the vncviewer conect to `localhost:590`, this connects to the SSH tunnel, which forwards traffic securely to the VNC server running on your VM.


Now through vncviewer open a terminal and launch RViz:
```bash
ros2 run rviz2 rviz2 & sleep 2 && ros2 param set /rviz use_sim_time true
```

## 6. Good to know stuff about Docker and more

a. How to get another terminal in a running docker container:
   ```bash
   docker exec -it isaac-sim-New bash
   ```

b. Remove docker image:
   ```bash
   docker rm isaac-sim-New
   ``` 

c. Check docker images:
   ```bash
   docker images
   ``` 

   ```bash
   docker ps
   ``` 

d. source install and colcon build mapping container:
   ```bash
   source ~/ERC-Mars-Rover/install/setup.bash


   colcon build --parallel-workers $(nproc) --symlink-install \
      --event-handlers console_direct+ --base-paths src \
      --packages-skip livox_ros_driver2 probe_detection \
      --cmake-args \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined" \
         --no-warn-unused-cli
   ```

e. How to transfer files from local pc to VM and VM to docker container:
   Local pc to VM:
   ```bash
   scp -r /home/<your_folder_> ubuntu@10.92.1.zzz:/home/ubuntu/<VM_folder_>
   ``` 

   VM to docker container:
   ```bash
   docker cp /path/to/folder container_name_or_id:/path/in/container
   ``` 
   e.g.:
   ```bash
   docker cp /home/ubuntu/01_MY_MODEL-20250319T134030Z-001/ isaac-sim:/isaac-sim-New/assets
   ``` 

f. How to save docker image of running container:
   ```bash
   docker ps
   ```

   ```bash
   docker commit <container_id_or_name> <new_image_name>:<tag>
   ``` 