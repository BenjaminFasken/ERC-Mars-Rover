from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation',
            executable='cmd_repeater.py',
            name='cmd_repeater'
        ),
        Node(
            package='gpio_controller',
            executable='gpio_node.py',
            name='gpio_node'
        )
    ])
