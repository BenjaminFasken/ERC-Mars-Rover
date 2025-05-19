#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class CmdVelRepublisher(Node):
    def __init__(self):
        super().__init__('fast_rtps_to_cyclone')
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
        
        # Republish "/firmware/battery_averaged" as "/firmware/battery_averaged_cyclone"
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(
            Float32,
            '/firmware/battery_averaged',
            self.battery_callback,
            qos_profile
        )
        self.battery_publisher = self.create_publisher(
            Float32,
            '/firmware/battery_averaged_cyclone',
            qos_profile
        )
        
    def cmd_vel_callback(self, msg):
        self.get_logger().info('Received cmd_vel_cyclone message')
        self.publisher.publish(msg)
        self.get_logger().info('Republished to cmd_vel')
        
        
    def battery_callback(self, msg):
        self.get_logger().info('Received battery_averaged message')
        self.battery_publisher.publish(msg)
        self.get_logger().info('Republished to battery_averaged_cyclone')

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