#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRepublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_republisher')
        self.subscriber = self.create_subscription(
            Twist,
            '/cmd_vel_cyclone',
            self.cmd_vel_callback,
            10
        )
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.get_logger().info('CmdVelRepublisher node started')

    def cmd_vel_callback(self, msg):
        self.get_logger().info('Received cmd_vel_cyclone message')
        self.publisher.publish(msg)
        self.get_logger().info('Republished to cmd_vel')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()