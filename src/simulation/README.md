# Simulation Environment for the Leo Rover with Sensors and Mars Terrain

This repository contains a simulation environment developed in **NVIDIA Isaac Sim**, featuring a digital model of the **Leo Rover**—both with and without sensors— in a Mars terrain mad from a 3D scan of a previous version of the **ERC Mars Yard**. The setup is fully integrated with **ROS2** and supports sensor configurations including:

- Stereolab’s **ZED X** stereo camera  
- **Hesai XT-32** LiDAR  
- **Ouster OS1** 32-channel LiDAR  

The environment allows realistic simulation of a sensor-equipped Leo Rover in Martian-like terrain, with **RTAB-Map** SLAM. All components can be launched from a single launch file for ease of use. The repository includes:

- Launch file for standard Isaac Sim
- Launch files for Isaac Sim with the sensorized Leo Rover in the Martian terrain
- Launch file Isaac Sim with the sensorized Leo Rover in the Martian terrain and the integration with RTAB-Map and the ZED wrapper

To ensure a smooth and reproducible experience, the entire environment is **containerized using Docker**, minimizing dependency issues. The setup is designed to run on a **virtual machine (VM)** using the **Strato HPC service**, available to **AAU students**.

## 📖 Documentation Overview

1. **Using Strato**  
   Instructions on how to access and use AAU's Strato high-performance computing platform.

2. **Building Docker Images**  
   Guide for creating and managing the necessary Docker containers.

3. **Launching the Full Simulation**  
   How to launch Isaac Sim with the sensorized Leo Rover, ZED wrapper, and RTAB-Map from a unified launch file, and visulize the simulation remotely.

4. **Custom Launch Options**  
   Running specific configurations such as:
   - Isaac Sim alone
   - Isaac Sim with the rover in Martian terrain
   - Other combinations using Python launch files

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

If you are new to Strato go to https://hpc.aau.dk/strato/ to get an overview and Introduction.

Compleat Initial Openstack setup (https://hpc.aau.dk/strato/getting-started/launch-instance/):
- Make SSH rules
- Create SSH key pair
- Upload key pair to OpenStack

The above you only have to do once. Then you can:
- Launch Ubuntu instance

Note: If you cheched [NO] to **Delete Volume on Instance Delete**, then you can delete your Instance, but your data is saved in the Volume. You can then create a new Instance based on the Volume, and the new Instance will be identical to the one you deleted with the exeption of a new IP-adress and "reboot".

### 1.1 Connecting to your Strato instance
To connec to your Strato instance, use the following SSH comand in your terminal on your local PC and replace strato_ssh_key with your strato shh key name and replace 10.92.1.179 with your instance's IP adress:
ssh -X -i ~/.ssh/strato_ssh_key ubuntu@10.92.1.179

### 1.1 Connecting to Your Strato Instance

To connect to your Strato instance, use the following SSH command in the terminal (on your local PC). Replace `/.ssh/strato_ssh_key` with the name of your SSH key file, and replace `10.92.1.179` with the actual IP address of your Strato instance:

```bash
ssh -X -i ~/.ssh/strato_ssh_key ubuntu@10.92.1.179
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

## 1. Building Docker Images

### 1.0 Install Docker

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

### 1.1 Building Docker Images

#### Option 1: Let the convenience script build the images for you
Navigate to the repository directory:
```bash
cd ~/ERC-Mars-Rover
```

Run the script:
```bash
./run_sim.sh
```

#### Option 2: Build the Images (optional)

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
