from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Path to the nav2_bringup package
    nav2_bringup_path = os.path.join(
        FindPackageShare('nav2_bringup').find('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    # Path to the yovio.launch.py file
    yovio_launch_path = os.path.join(
        FindPackageShare('slam').find('slam'),
        'launch',
        'yovio.launch.py'
    )
    

    return LaunchDescription([
        # Include yovio.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yovio_launch_path)
        ),

        # Launch frontier_exploration explorer after yovio is fully up
        ExecuteProcess(
            cmd=['ros2', 'run', 'frontier_exploration', 'explorer'],
            output='screen',
            additional_env={'WAIT_FOR_YOVIO': 'true'}
        ),

        # Include nav2_bringup navigation_launch.py after yovio is fully up
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_path),
            launch_arguments={'wait_for_yovio': 'true'}.items()
        ),
    ])