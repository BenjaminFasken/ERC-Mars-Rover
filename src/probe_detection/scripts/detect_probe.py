#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces.msg import ProbeSegmentation
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')
        
        # Subscriber for input image
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10)
        
        # Publisher for segmented image
        self.image_publisher = self.create_publisher(
            Image,
            '/probe_detector/segmented_image',
            10)
        
        # Publisher for detection data
        self.detection_publisher = self.create_publisher(
            ProbeSegmentation,
            '/probe_detector/segmentations',
            10)
        
        self.bridge = CvBridge()
        
        # Get the package share directory and construct the model path
        package_name = 'probe_detection'
        try:
            package_share_dir = get_package_share_directory(package_name)
            self.model_path = os.path.join(package_share_dir, 'models', 'probe_segmentation.pt')
        except Exception as e:
            self.get_logger().error(f"Failed to find package share directory: {e}")
            raise

        # Verify the file exists
        self.get_logger().info(f"Looking for model at: {self.model_path}")
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Model file not found at: {self.model_path}")
            raise FileNotFoundError(f"Model file not found at: {self.model_path}")
        
        # Load YOLO model
        self.model = YOLO(self.model_path)
        self.get_logger().info(f"Loaded YOLO model from {self.model_path}")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            if msg.encoding == "bgra8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgra8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convert BGRA to BGR if needed
            if cv_image.shape[2] == 4:
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)

            # Rotate image 180 degrees
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)

            # Run YOLO inference
            results = self.model.predict(
                source=cv_image,
                imgsz=416,
                conf=0.25,
                device=0
            )
            
            # Get annotated image
            segmented_image = results[0].plot()
            
            # Publish segmented image
            segmented_msg = self.bridge.cv2_to_imgmsg(segmented_image, encoding="bgr8")
            self.image_publisher.publish(segmented_msg)

            # Process and publish detection data
            if results[0].boxes is not None and results[0].masks is not None:
                for i, (box, mask) in enumerate(zip(results[0].boxes, results[0].masks)):
                    detection_msg = ProbeSegmentation()
                    detection_msg.id = i
                    detection_msg.class_name = self.model.names[int(box.cls)]
                    detection_msg.confidence = float(box.conf)
                    detection_msg.box = [float(coord) for coord in box.xyxy[0]]
                    mask_data = mask.data[0].cpu().numpy()
                    detection_msg.mask = mask_data.flatten().tolist()
                    detection_msg.mask_width = mask_data.shape[1]
                    detection_msg.mask_height = mask_data.shape[0]
                    self.detection_publisher.publish(detection_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()