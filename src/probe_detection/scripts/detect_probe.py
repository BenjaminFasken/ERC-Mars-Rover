#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import cv2
import os
import threading
import sys
import termios
import tty
import numpy as np

class ZEDImageVisualizer(Node):
    def __init__(self):
        super().__init__('zed_image_visualizer')
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.visualizing = False
        self.running = True
        
        # Load the trained YOLO model (adjust path to your trained weights)
        #self.model_path = '/home/daroe/ERC-Mars-Rover/runs/detect/train14/weights/best.pt'  # Update this!
        self.model_path = '/home/daroe/ERC-Mars-Rover/src/probe_detection/models/many_probes_weights.pt'  # Update this!

        self.model = YOLO(self.model_path)
        self.get_logger().info(f"Loaded YOLO model from {self.model_path}")

        self.get_logger().info("Press SPACE to start/pause visualization, ESC to exit.")
        threading.Thread(target=self.key_listener, daemon=True).start()

    def key_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self.running:
                key = sys.stdin.read(1)
                if key == ' ':  # Space key pressed
                    self.visualizing = not self.visualizing
                    state = "Visualizing" if self.visualizing else "Paused"
                    self.get_logger().info(f"{state} model detections...")
                elif key == '\x1b':  # ESC key pressed
                    self.get_logger().info("Exiting...")
                    self.running = False
                    cv2.destroyAllWindows()
                    rclpy.shutdown()
                    sys.exit(0)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def image_callback(self, msg):
        if not self.visualizing:
            return

        try:
            # Convert ROS Image to OpenCV format
            if msg.encoding == "bgra8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgra8")
            else:
                self.get_logger().warning(f"Unexpected encoding: {msg.encoding}, falling back to bgr8")
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convert BGRA to BGR (YOLO expects BGR or RGB, not BGRA)
            if cv_image.shape[2] == 4:  # Check if image has 4 channels (BGRA)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)

            #Rotate image 180 degrees
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)

            # Run YOLO inference
            results = self.model.predict(
                source=cv_image,
                imgsz=416,  # Match your training size
                conf=0.25,  # Confidence threshold
                device=0    # Use GPU
            )

            # Draw detections on the image
            annotated_image = results[0].plot()  # Get the annotated image with boxes/labels

            # Display the image with detections
            cv2.imshow("YOLO Detections", annotated_image)
            cv2.waitKey(1)  # Update the window (1ms delay)

            # Log some info about detections
            num_detections = len(results[0].boxes)
            #self.get_logger().info(f"Detected {num_detections} objects in frame")

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ZEDImageVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()