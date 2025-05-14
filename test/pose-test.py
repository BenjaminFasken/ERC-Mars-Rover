import pyzed.sl as sl
import time
import sys
from datetime import datetime
import math # For Euler angles conversion if needed manually

def main():
    # --- ZED Camera Initialization ---
    zed = sl.Camera()
    init_params = sl.InitParameters()

    # --- Settings for Positional Tracking ---
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP # Or choose your preferred system
    # Use a depth mode compatible with tracking (NEURAL recommended for ZED2i if GPU available)
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
    # init_params.depth_mode = sl.DEPTH_MODE.ULTRA # Alternative good option
    # init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE # Lower accuracy alternative
    init_params.camera_resolution = sl.RESOLUTION.HD1080  # Set resolution to 1080p
    init_params.camera_fps = 30                           # Moderate FPS

    # !!! IMPORTANT: Enable positional tracking !!!
    #init_params.enable_positional_tracking = True
    # Optional but recommended for ZED 2i: Ensure IMU fusion is used
    init_params.sensors_required = True # ZED 2i uses Visual-Inertial SLAM

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open ZED camera: {err}")
        zed.close()
        sys.exit(1)
    else:
        print("ZED camera opened successfully.")
        print(f"Using Coordinate System: {init_params.coordinate_system}")
        print(f"Using Depth Mode: {init_params.depth_mode}")

    # --- Enable Positional Tracking Module ---
    # Positional tracking parameters (can be default or customized)
    tracking_parameters = sl.PositionalTrackingParameters()
    # E.g., enable IMU fusion explicitly if needed, though usually handled by sensors_required
    # tracking_parameters.enable_imu_fusion = True
    # E.g., set initial world pose if starting from a known location
    # initial_position = sl.Transform()
    # initial_position.set_translation(sl.Translation(0,0,0)) # Start at origin
    # tracking_parameters.initial_world_transform = initial_position
    err = zed.enable_positional_tracking(tracking_parameters)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to enable positional tracking: {err}")
        zed.close()
        sys.exit(1)
    else:
        print("Positional tracking enabled successfully.")


    # --- Prepare Runtime and Pose Objects ---
    runtime_params = sl.RuntimeParameters()
    # Optional: Enable depth sensing if you want to use it alongside tracking
    # runtime_params.enable_depth = True

    pose_data = sl.Pose() # Structure to hold the pose data

    print("\n--- Starting Positional Tracking Stream ---")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            # Grab a new frame - REQUIRED for positional tracking updates
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:

                # --- Get the current calculated pose ---
                # Pass the sl.Pose() object and specify the reference frame
                # REFERENCE_FRAME.WORLD gives the pose relative to the starting point (or map origin)
                tracking_status = zed.get_position(pose_data, sl.REFERENCE_FRAME.WORLD)

                # Check if tracking is stable and data is valid
                if tracking_status == sl.POSITIONAL_TRACKING_STATE.OK:
                    # --- Access Pose Data ---

                    # Translation (Position: x, y, z)
                    translation = pose_data.get_translation()
                    # tx = translation.get()[0]
                    # ty = translation.get()[1]
                    # tz = translation.get()[2]
                    ty = -translation.get()[0]
                    tz = translation.get()[1]
                    tx = -translation.get()[2]

                    # Orientation (as Euler Angles - Roll, Pitch, Yaw in degrees)
                    # Easier to interpret than quaternions initially
                    # The function returns [Roll, Pitch, Yaw]
                    euler_angles = pose_data.get_euler_angles(radian=False) # Get in degrees
                    # roll = euler_angles[0]
                    # pitch = euler_angles[1]
                    # yaw = euler_angles[2]
                    pitch = euler_angles[0]
                    yaw = euler_angles[1]
                    roll = euler_angles[2]

                    # # --- OR ---
                    # # Orientation (as Quaternion: x, y, z, w)
                    # orientation_quat = pose_data.get_orientation()
                    # ox = orientation_quat.get()[0]
                    # oy = orientation_quat.get()[1]
                    # oz = orientation_quat.get()[2]
                    # ow = orientation_quat.get()[3]

                    timestamp_ns = pose_data.timestamp.get_nanoseconds() # Get timestamp in nanoseconds

                    # Print the pose data
                    print(f"Timestamp: {timestamp_ns} | Status: {tracking_status} | "
                          f"Pos (m): X={tx:.3f}, Y={ty:.3f}, Z={tz:.3f} | "
                          f"Orient (deg): Roll={roll:.2f}, Pitch={pitch:.2f}, Yaw={yaw:.2f}")
                    # Write pose data to a CSV file
                    # Write pose data to a new CSV file based on the current time

                    # Generate a unique file name based on the current time
                    


                    # Append pose data to the file
                    with open(file_path, "a") as file:
                        file.write(f"{timestamp_ns},{tx:.3f},{ty:.3f},{tz:.3f},{roll:.2f},{pitch:.2f},{yaw:.2f}\n")

                else:
                    # Tracking is not stable or initializing
                    print(f"Tracking Status: {tracking_status}")
                    # Optional: You might want to clear or reset pose variables here if needed

            else:
                 print("Failed to grab frame")
                 # Optional: break or wait

            # Optional: Control loop rate (tracking updates happen internally at camera FPS)
            # A short sleep prevents this loop from consuming 100% CPU if grab is fast
            time.sleep(0.01) # e.g., sleep 10ms

    except KeyboardInterrupt:
        print("\n--- Stopping stream ---")

    finally:
        # --- Cleanup ---
        print("Disabling positional tracking...")
        zed.disable_positional_tracking()
        print("Closing ZED camera...")
        zed.close()
        print("ZED camera closed.")

if __name__ == "__main__":
    current_time = datetime.now().astimezone().strftime("%Y%m%d_%H%M")
    file_path = f"/home/robotlab/Test/LocData/pose_data_5min_{current_time}.csv"
    file_header = "Timestamp (ns),X (m),Y (m),Z (m),Roll (deg),Pitch (deg),Yaw (deg)\n"
    with open(file_path, "w") as file:
        file.write(file_header)
    main()