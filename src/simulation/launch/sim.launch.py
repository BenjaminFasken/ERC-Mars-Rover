import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    camera_model = DeclareLaunchArgument(
        'camera_model',
        default_value='zedx',
        description='ZED camera model to use'
    )
    
    sim_mode = DeclareLaunchArgument(
        'sim_mode',
        default_value='true',
        description='Enable simulation mode'
    )
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # Path to Isaac Sim directory
    isaac_sim_path = "../isaac-sim"

    # Execute Isaac Sim Python script without logging
    isaac_sim_command = ExecuteProcess(
        cmd=['bash', '-c', 'pwd && ./python.sh assets/leorover_xt32_lifted_30fps.py'],
        cwd=isaac_sim_path,
        output='screen',
        shell=True
    )

    # ZED camera launch
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'sim_mode': LaunchConfiguration('sim_mode'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Create launch description
    return LaunchDescription([
        # Launch arguments
        camera_model,
        sim_mode,
        use_sim_time,
        
        # Log info for debugging
        LogInfo(msg='Launching Isaac Sim script'),
        
        # Isaac Sim script execution
        isaac_sim_command,
        
        # Delayed ZED launch after fixed time
        TimerAction(
            period=50.0,  # Wait 50 sec before launching ZED
            actions=[
                LogInfo(msg='Starting ZED wrapper after delay...'),
                zed_launch
            ]
        ),
    ])