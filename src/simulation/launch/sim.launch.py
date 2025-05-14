import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition

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
    isaac_sim_path = "/isaac-sim"

    # Log file and readiness marker. These are used to make the launch of the ZED camera dependent on the completion of the Isaac Sim script.
    log_file = "/tmp/isaac_sim_log.txt"
    ready_file = "/tmp/isaac_sim_setup_ready"

    # Execute Isaac Sim Python script with output to log file
    isaac_sim_command = ExecuteProcess(
        cmd=['bash', '-c', f'./python.sh assets/leorover_os1_lifted_30fps.py 2>&1 | tee {log_file}'],
        cwd=isaac_sim_path,
        output='screen',
        shell=True
    )

    # Periodically check for readiness marker file
    check_ready_command = ExecuteProcess(
        cmd=['bash', '-c', f'test -f {ready_file} || exit 0'],
        output='screen',
        shell=True
    )

    # ZED camera launch (only execute if ready_file exists)
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
        }.items(),
        condition=UnlessCondition(Command(['test -f ', ready_file, ' && echo 0 || echo 1']))
    )

    # Create launch description
    return LaunchDescription([
        # Launch arguments
        camera_model,
        sim_mode,
        use_sim_time,
        
        # Log info for debugging
        LogInfo(msg='Launching Isaac Sim script and waiting for setup completion'),
        
        # Isaac Sim script execution
        isaac_sim_command,
        
        # Periodically check for readiness
        TimerAction(
            period=5.0,  # Check every 5 seconds
            actions=[check_ready_command]
        ),
        
        # ZED camera launch
        zed_launch,
        
        # Log completion
        TimerAction(
            period=10.0,  # Wait longer to ensure ZED launch is complete
            actions=[
                LogInfo(msg='Launch process completed')
            ]
        )
    ])