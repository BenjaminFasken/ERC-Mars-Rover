#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import numpy as np

class ZEDImageSaver(Node):
    def __init__(self):
        super().__init__('zed_image_saver')
        
        # Default path
        self.default_path = "datasets/raw_images/OnRover"
        self.save_path = self.check_and_set_path(self.default_path)
        
        # ROS2 Subscriptions
        self.image_topic = '/zed/zed_node/rgb/image_rect_color'
        self.control_topic = '/zed_image_saver/control'
        
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)
        self.control_sub = self.create_subscription(
            Bool,
            self.control_topic,
            self.control_callback,
            10)
        
        self.bridge = CvBridge()
        self.capturing = False  # Start in inactive state
        self.image_count = 0
        self.images_seen = 0
        self.skip_every_n_image = 3
        
        # Log subscription and save path info
        self.get_logger().info("ZED Image Saver initialized")
        self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")
        self.get_logger().info(f"Subscribing to control topic: {self.control_topic}")
        self.get_logger().info(f"Saving images to: {self.save_path}")
        self.get_logger().info('Run the following command to start capturing images:')
        self.get_logger().info(f'ros2 topic pub {self.control_topic} std_msgs/Bool "data: true"')
        self.get_logger().info('Run the following command to stop capturing images:')
        self.get_logger().info(f'ros2 topic pub {self.control_topic} std_msgs/Bool "data: false"')
        self.get_logger().info("Starting in inactive state")

    def check_and_set_path(self, path):
        """Check if path exists, create it if possible, or fall back to raw_images"""
        try:
            os.makedirs(path, exist_ok=True)
            return path
        except Exception as e:
            self.get_logger().warn(f"Could not create/use path {path}: {e}")
            fallback_path = "raw_images"
            os.makedirs(fallback_path, exist_ok=True)
            self.get_logger().info(f"Falling back to saving images in {fallback_path}")
            return fallback_path

    def control_callback(self, msg):
        """Callback for control command"""
        new_state = msg.data
        if new_state != self.capturing:
            self.capturing = new_state
            state_str = "Started capturing images..." if self.capturing else "Stopped capturing images..."
            self.get_logger().info(state_str)

    def image_callback(self, msg):
        if not self.capturing:
            return

        try:
            self.images_seen += 1
            
            # Skip frames if needed
            if (self.images_seen % self.skip_every_n_image) != 0:
                return

            # Verify image data
            if not msg.data or len(msg.data) == 0:
                self.get_logger().error("Image message data is empty!")
                return

            expected_size = msg.width * msg.height * 4  # 4 bytes per pixel for bgra8
            if len(msg.data) != expected_size:
                self.get_logger().error(f"Data size mismatch! Expected {expected_size} bytes, got {len(msg.data)} bytes")
                return

            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgra8" if msg.encoding == "bgra8" else "bgr8")
            
            if cv_image is None or not isinstance(cv_image, np.ndarray):
                self.get_logger().error("Image conversion failed!")
                return

            # Save the image
            filename = os.path.join(self.save_path, f"frame_{self.image_count:06d}.jpg")
            cv2.imwrite(filename, cv_image)
            full_path = os.path.abspath(filename)  # Get the absolute path
            self.get_logger().info(f"Saved image to: {full_path}")
            
            self.image_count += 1

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error saving image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ZEDImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()