#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, UInt8, Header
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from interfaces.msg import ProbeLocations
import os
from cv2 import imread, IMREAD_GRAYSCALE
from cv_bridge import CvBridge

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.counter = 0

        # Publishers
        self.pub_status = self.create_publisher(Bool, '/rover/status', 10)
        self.pub_position = self.create_publisher(Vector3, '/rover/position', 10)
        self.pub_attitude = self.create_publisher(Vector3, '/rover/attitude', 10)
        self.pub_velocity = self.create_publisher(Vector3, '/rover/velocity', 10)
        self.pub_probes = self.create_publisher(ProbeLocations, '/rover/probes', 10)
        self.pub_image = self.create_publisher(Image, '/rover/image', 10)
        self.pub_battery = self.create_publisher(Float32, '/rover/battery', 10)
        self.pub_mode = self.create_publisher(UInt8, '/rover/mode', 10)

        # Timers
        self.timer_status = self.create_timer(1.0, self.publish_status)
        self.timer_position = self.create_timer(0.5, self.publish_position)
        self.timer_attitude = self.create_timer(0.7, self.publish_attitude)
        self.timer_velocity = self.create_timer(0.6, self.publish_velocity)
        self.timer_probes = self.create_timer(2.0, self.publish_probes)
        self.timer_battery = self.create_timer(1.5, self.publish_battery)
        self.timer_mode = self.create_timer(3.0, self.publish_mode)

        self.get_logger().info('Started TestPublisher')

    def publish_status(self):
        msg = Bool()
        msg.data = (self.counter % 2 == 0)
        self.pub_status.publish(msg)
        self.get_logger().info(f"Published status: {msg.data}")

    def publish_position(self):
        msg = Vector3()
        msg.x = 1.0 + self.counter * 0.1
        msg.y = 2.0 + self.counter * 0.1
        msg.z = 0.5 + self.counter * 0.05
        self.pub_position.publish(msg)
        self.get_logger().info(f"Published position: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})")

    def publish_attitude(self):
        msg = Vector3()
        msg.x = 0.1 + self.counter * 0.01
        msg.y = 0.2 + self.counter * 0.01
        msg.z = 0.3 + self.counter * 0.01
        self.pub_attitude.publish(msg)
        self.get_logger().info(f"Published attitude: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})")

    def publish_velocity(self):
        msg = Vector3()
        msg.x = 0.4 + self.counter * 0.01
        msg.y = 0.5 + self.counter * 0.01
        msg.z = 0.1 + self.counter * 0.005
        self.pub_velocity.publish(msg)
        self.get_logger().info(f"Published velocity: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})")

    def publish_probes(self):
        msg = ProbeLocations()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.num_probes = 5
        # Cap the confidence to ensure it stays in [0.0, 1.0]
        confidence = max(0.0, min(1.0, 0.9 - self.counter * 0.05))
        msg.classification_confidence = [
            confidence for _ in range(msg.num_probes)
        ]
        msg.probes = [
            0.0 + self.counter * 0.2, 20.0, 0.0,
            1.0 + self.counter * 0.2, 21.0, 0.1,
            2.0 + self.counter * 0.2, 22.0, 0.2,
            3.0 + self.counter * 0.2, 23.0, 0.3,
            4.0 + self.counter * 0.2, 24.0, 0.4
        ]
        self.pub_probes.publish(msg)
        self.get_logger().info(
            f"Published probes: {[f'({msg.probes[i]:.2f}, {msg.probes[i+1]:.2f}, {msg.probes[i+2]:.2f})' for i in range(0, msg.num_probes * 3, 3)]}, "
            f"confidences={msg.classification_confidence[:msg.num_probes]}, counter={self.counter}"
        )
        # Reset counter to keep confidences in a reasonable range
        self.counter = (self.counter + 1) % 10  # Reset every 10 cycles

    def publish_image(self):

        # Paths to the images
        image1_path = "/home/daroe/ERC-Mars-Rover/src/gcs/gcs_gui/pictures/image1.png"
        image2_path = "/home/daroe/ERC-Mars-Rover/src/gcs/gcs_gui/pictures/image2.png"

        # Determine which image to send
        image_path = image1_path if self.counter % 2 == 0 else image2_path

        # Load the image
        if not os.path.exists(image_path):
            self.get_logger().error(f"Image file not found: {image_path}")
            return

        cv_image = imread(image_path, IMREAD_GRAYSCALE)
        if cv_image is None:
            self.get_logger().error(f"Failed to load image: {image_path}")
            return

        # Convert to ROS Image message
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(cv_image, encoding="mono8")
        msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the image
        self.pub_image.publish(msg)
        self.get_logger().info(f"Published image: {image_path}")

    def publish_battery(self):
        msg = Float32()
        msg.data = 16.50- self.counter * 0.1
        self.pub_battery.publish(msg)
        self.get_logger().info(f"Published battery: {msg.data:.2f}")

    def publish_mode(self):
        msg = UInt8()
        msg.data = self.counter % 4
        self.pub_mode.publish(msg)
        self.get_logger().info(f"Published mode: {msg.data}")
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()