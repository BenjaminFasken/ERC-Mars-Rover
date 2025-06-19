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

    # Command to run a Python HTTP server on port 3042 with delay
    http_server_command = TimerAction(
        period=10.0,  # 5 second delay
        actions=[
            ExecuteProcess(
                cmd=["cd", "install/gcs/lib/gcs", "&&", "python3", "-m", "http.server", "4269"],
                output="screen",
                shell=True
            )
        ]
    )

    # ROS 2 node for GCSServerListener with delay
    gcs_listener_node = TimerAction(
        period=3.0,  # 15 second delay
        actions=[
            Node(
                package="gcs",
                executable="webview_app.py",
                name="webview_app",
                output="screen"
            )
        ]
    )
    # Command to run RViz2 with specified configuration
    rviz_command = ExecuteProcess(
        cmd=["ros2", "run", "rviz2", "rviz2", "-d", "src/gcs/rviz2/rviz-config.rviz"],
        output="screen",
        shell=True
    )
    # Create and return the launch description
    return LaunchDescription([
        http_server_command,
        ros_web_bridge_command,
        gcs_listener_node,
        rviz_command
    ])