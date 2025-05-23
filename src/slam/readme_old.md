### Installing Livox 2 from Scratch

1. Clone the Livox SDK repository:
    ```sh
    cd ~
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd Livox-SDK2/
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    ```

2. The installation will place files in the following locations:
    - `/usr/local/lib/liblivox_lidar_sdk_static.a`
    - `/usr/local/include/livox_lidar_def.h`
    - `/usr/local/include/livox_lidar_api.h`
    - `/usr/local/include/livox_lidar_cfg.h`
    - `/usr/local/lib/liblivox_lidar_sdk_shared.so`

### Compiling the ROS2 Workspace

1. Source ROS:
    ```sh
    source /opt/ros/humble/setup.bash
    ```

2. Clone the Livox ROS driver repository:
    ```sh
    cd /path/to/your/ws/
    git clone https://github.com/Livox-SDK/Livox_ros_driver2.git src/livox_ros_driver2
    cd src/livox_ros_driver2
    ```

3. Modify the `MID360_config.json` file:
    - Set `cmd_data_ip`, `push_msg_ip`, `point_data_ip`, and `imu_data_ip` to `192.168.1.50`
    - Set `lidar_config_ip` to `192.168.1.xxx` (follow the guide for your lidar)
    - Ensure `xfer_format` is set to `0` in the launch script

4. Build the workspace:
    ```sh
    ./build.sh humble 
    export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
    ```

### Running the Livox Program

1. Source the workspace:
    ```sh
    cd /path/to/your/ws/
    source install/setup.bash
    ```

2. Launch the Livox ROS driver:
    ```sh
    ros2 launch livox_ros_driver2 rviz_MID360_launch.py
    ```

**Note:** Ensure the subnet (192.168.x.0) of the lidar is not the same as the Wi-Fi if you want to access the computer through SSH while the lidar is running.

### Installing the ZED ROS2 Wrapper and Examples

1. **Install the ZED ROS2 Wrapper and its dependencies:**
    - Follow the instructions provided in the [ZED ROS2 Wrapper GitHub repository](https://github.com/stereolabs/zed-ros2-wrapper).

2. **Install the ZED ROS2 Examples:**
    - Follow the build instructions provided in the [ZED ROS2 Examples GitHub repository](https://github.com/stereolabs/zed-ros2-examples#build-the-package).

3. **Test the Camera:**
    - After the wrapper is installed, you can test the camera with the following command:
        ```sh
        ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
        ```

4. **Visualize the Camera Output:**
    - To visualize the camera output, use the following command:
        ```sh
        ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zed2i
        ```

### Install RTAB-map
**Install RTAB-map for ROS2:**
    - Follow the instructions provided in the [RTAB-map ROS2 GitHub repository](https://github.com/introlab/rtabmap_ros/tree/ros2#rtabmap_ros).

### Testing Sensors in RTAB-map

Ensure you have downloaded and installed the drivers and wrappers for the individual sensors, and have tested them.
#### Test LiDAR in RTAB-map

To test the LiDAR and provide visual output, first use the following command:
```sh
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```
This should publish data to the topics `/livox/lidar` and `/livox/imu`. Then use the following command to integrate and visualize it in RTAB-map:
```sh
ros2 launch rtabmap_examples lidar3d.launch.py lidar_topic:=/livox/lidar frame_id:=livox_frame imu_topic:=/livox/imu
```

#### Test ZED2i in RTAB-map

To test the ZED2i camera and provide visual output, use the following command:
```sh
ros2 launch rtabmap_examples zed.launch.py camera_model:=zed2i
```

### Run Lidar and Zed2i Together in RTAB-map

To combine the Livox Mid-360 LiDAR and ZED2i camera for 3D mapping with RTAB-Map, follow these steps. This setup uses `rgbd_sync` to synchronize the ZED2i’s RGB-D data and `lidar3d_assemble.launch.py` to integrate it with LiDAR scans.

#### Steps

cd into/the/ws
cd src
source the ws

1. **Start the Livox Driver:**
    ```sh
    ros2 launch slam rviz_MID360_launch.py
    ```
    - The Lidar should publish:
    - `/livox/lidar`
    - `/livx/imu`

2. **Start the ZED2i Camera:**
    ```sh
    ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
    ```
    - The ZED2i should publish:
    - `/zed/zed_node/rgb/image_rect_color`
    - `/zed/zed_node/depth/depth_registered`
    - `/zed/zed_node/rgb/camera_info`

3. **Publish Static Transforms:**
    - Define `base_link` as the root frame, connecting the LiDAR and ZED2i:
        ```sh
        ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link livox_frame
        ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link zed_left_camera_frame
        ```
    - Adjust the translation based on your sensor placement (x, y ,z, yaw, pitch, roll), in our case:
        ```sh
        ros2 run tf2_ros static_transform_publisher 0.0508870 -0.0001280 0.0517150 -0.7854 0 0 base_link livox_frame 
        ros2 run tf2_ros static_transform_publisher 0.1865610 -0.0601020 0.00365 0 0.2618 -3.1416 base_link zed_left_camera_frame 
        ```

4. **Synchronize ZED2i RGB-D Data:**
    ```sh
    ros2 run rtabmap_sync rgbd_sync --ros-args \
    -r rgb/image:=/zed/zed_node/rgb/image_rect_color \
    -r depth/image:=/zed/zed_node/depth/depth_registered \
    -r rgb/camera_info:=/zed/zed_node/rgb/camera_info \
    -r rgbd_image:=/rgbd_image
    ```

5. **Launch LiDAR and RGB-D Assembly:**
    ```sh
    ros2 launch rtabmap_examples lidar3d_assemble.launch.py \
    frame_id:=base_link \
    lidar_topic:=/livox/lidar \
    imu_topic:=/livox/imu \
    rgbd_image_topic:=/rgbd_image \
    voxel_size:=0.05 \
    assembling_time:=1.0
    ```

6. **Visualize the Map:**
    ```sh
    ros2 run rviz2 rviz2
    ```
    - Add displays:
        - `PointCloud2`: `/livox/lidar`
        - `RGBDImage`: `/rgbd_image`
        - `PointCloud2`: `/assembled_cloud`
        - `Odometry`: `/icp_odom`
        - `Map`: `/rtabmap/map`

#### Notes
- **TF Calibration:** The static transforms assume the LiDAR is 10 cm above the ZED2i. Adjust these based on your physical setup.
- **Performance Tuning:** If mapping is slow, increase `voxel_size` (e.g., `0.1`) or reduce `assembling_time` (e.g., `0.5`).

