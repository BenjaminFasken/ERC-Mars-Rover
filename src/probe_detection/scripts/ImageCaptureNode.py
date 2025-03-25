#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import threading
import sys
import termios
import tty
import numpy as np


path = "datasets/raw_images/Final"

class ZEDImageSaver(Node):
    def __init__(self):
        super().__init__('zed_image_saver')
        self.subscription = self.create_subscription(  
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.capturing = False
        self.image_count = 0
        self.images_seen = 0
        self.running = True
        self.skip_every_n_image = 3
        self.create_directories()
        self.get_logger().info("Press SPACE to start/pause image capture, ESC to exit.")
        threading.Thread(target=self.key_listener, daemon=True).start()

    def create_directories(self): 
        os.makedirs("raw_images", exist_ok=True)  # Single directory for all images

    def key_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self.running:
                key = sys.stdin.read(1)
                if key == ' ':  # Space key pressed
                    self.capturing = not self.capturing
                    state = "Capturing" if self.capturing else "Paused"
                    self.get_logger().info(f"{state} images...")
                elif key == '\x1b':  # ESC key pressed
                    self.get_logger().info("Exiting...")
                    self.running = False
                    rclpy.shutdown()
                    sys.exit(0)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def image_callback(self, msg):
        self.get_logger().info(f"Received Image Message:\n"
                               f" - Header: {msg.header}\n"
                               f" - Width: {msg.width}, Height: {msg.height}\n"
                               f" - Encoding: {msg.encoding}\n"
                               f" - Step: {msg.step} bytes\n")

        if not self.capturing:
            return

        
        try:
            self.images_seen += 1
            # Ensure the data field is not empty
            if not msg.data or len(msg.data) == 0:
                self.get_logger().error("Image message data is empty!")
                return

            # Verify data size matches expected dimensions
            expected_size = msg.width * msg.height * 4  # 4 bytes per pixel for bgra8
            if len(msg.data) != expected_size:
                self.get_logger().error(f"Data size mismatch! Expected {expected_size} bytes, got {len(msg.data)} bytes")
                return

            # Image is valid, check if 
            if (self.images_seen % self.skip_every_n_image) != 0:
                return

            # Convert ROS Image to OpenCV format
            self.get_logger().info(f"Converting image with encoding: {msg.encoding}")
            if msg.encoding == "bgra8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgra8")
            else:
                self.get_logger().warning(f"Unexpected encoding: {msg.encoding}, falling back to bgr8")
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Check if conversion was successful
            if cv_image is None:
                self.get_logger().error("CvBridge returned None!")
                return
            if not isinstance(cv_image, np.ndarray):
                self.get_logger().error(f"Converted image is not a NumPy array! Type: {type(cv_image)}")
                return

            self.get_logger().info(f"Image converted successfully. Shape: {cv_image.shape}, Type: {cv_image.dtype}")

            # Ensure the array is contiguous
            cv_image = np.ascontiguousarray(cv_image)
            self.get_logger().info(f"Ensured contiguous array. Shape: {cv_image.shape}, Type: {cv_image.dtype}")

            # Save the full image as-is (BGRA format)
            filename = os.path.join(path, f"frame_{self.image_count:06d}.jpg")
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Saved {filename}")
            self.image_count += 1

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error saving image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ZEDImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()