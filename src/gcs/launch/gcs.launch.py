from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    # Command to run YAMCS server
    ros_web_bridge_command = ExecuteProcess(
        cmd=["ros2", "run", "rosbridge_server", "rosbridge_websocket"],
        output="screen",
        shell=True
    )

    # ROS 2 node for GCSServerListener with delay
    gcs_listener_node = TimerAction(
        period=3.0,  # Delay in seconds
        actions=[
            Node(
                package="gcs",
                executable="webview_app.py",
                name="webview_app",
                output="screen"
            )
        ]
    )

    # Create and return the launch description
    return LaunchDescription([
        ros_web_bridge_command,
        gcs_listener_node
    ])