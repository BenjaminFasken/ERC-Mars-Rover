from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Define workspace and package paths
    workspace_path = os.path.expanduser("~/ERC-Mars-Rover")
    gcs_server_path = os.path.join(workspace_path, "src", "gcs", "gcs_server")

    # Command to run YAMCS server
    yamcs_command = ExecuteProcess(
        cmd=["mvn", "yamcs:run"],
        cwd=gcs_server_path,
        output="screen",
        shell=True
    )

    # ROS 2 node for GCSServerListener
    gcs_listener_node = Node(
        package="gcs",
        executable="GCSServerListener.py",
        name="gcs_server_listener",
        output="screen"
    )

    # Create and return the launch description
    return LaunchDescription([
        yamcs_command,
        gcs_listener_node
    ])