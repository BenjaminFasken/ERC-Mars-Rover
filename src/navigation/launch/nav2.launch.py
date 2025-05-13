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
    rtab_launch_path = os.path.join(
        FindPackageShare('slam').find('slam'),
        'launch',
        'rtab.launch.py'
    )
    
    
    
    return LaunchDescription([
        # Include yovio.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtab_launch_path)
        ),

        # # Launch frontier_exploration explorer
        ExecuteProcess(
            cmd=['ros2', 'run', 'frontier_exploration', 'explorer'],
            output='screen'
        ),

        # Launch the map_filter_nodes
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'slam', 'map_filter_node.py'],
        #     output='log'
        # ),
        
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'slam', 'path_node.py'],
        #     output='log'
        # ),
        
        # Launch probe filter
        ExecuteProcess(
            cmd=['ros2', 'run', 'navigation', 'probe_filtering_node'],
            output='log'
        ),
        
        #Build
        #Include nav2_bringup navigation_launch.py 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_path)
        ),
    ])