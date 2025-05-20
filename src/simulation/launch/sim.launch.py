import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

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

    # Log file and readiness marker
    log_file = "/tmp/isaac_sim_log.txt"
    ready_file = "/tmp/isaac_sim_setup_ready"
    
    # Clean up readiness and log files at startup to avoid stale files
    cleanup_command = ExecuteProcess(
        cmd=['bash', '-c', f'rm -f {ready_file} {log_file} /tmp/zed_log.txt'],
        output='screen',
        shell=True
    )

    # Execute Isaac Sim Python script with output to log file
    isaac_sim_command = ExecuteProcess(
        cmd=['bash', '-c', f'pwd && ./python.sh assets/leorover_os1_lifted_30fps.py 2>&1 | tee {log_file}'],
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

    # Function to check readiness file
    def check_ready():
        return IfCondition(
            Command(['bash -c "test -f /tmp/isaac_sim_setup_ready && echo 1 || echo 0"'])
        )

    # Create launch description
    return LaunchDescription([
        # Launch arguments
        camera_model,
        sim_mode,
        use_sim_time,
        
        # Clean up readiness and log files
        cleanup_command,
        
        # Log info for debugging
        LogInfo(msg='Launching Isaac Sim script and waiting for setup completion'),
        
        # Isaac Sim script execution
        isaac_sim_command,
        
        # Delayed ZED launch with periodic checking
        TimerAction(
            period=10.0,  # Check every 10 seconds
            actions=[
                LogInfo(msg='Checking for Isaac Sim readiness...'),
                ExecuteProcess(
                    cmd=['bash', '-c', 'ls -l /tmp/isaac_sim_setup_ready || echo "Readiness file not found"'],
                    output='screen',
                    shell=True
                ),
                zed_launch
            ],
            condition=check_ready()
        ),
        
        # Log completion with readiness status
        TimerAction(
            period=270.0, 
            actions=[
                LogInfo(msg='Launch process completed'),
                ExecuteProcess(
                    cmd=['bash', '-c', 'test -f /tmp/isaac_sim_setup_ready && echo "Isaac Sim ready" || echo "Isaac Sim not ready after 4.5 minutes"'],
                    output='screen',
                    shell=True
                )
            ]
        )
    ])