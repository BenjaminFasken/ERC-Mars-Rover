#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces.msg import ProbeSegmentation  # Ensure this is your custom message
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from typing import Tuple, List
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')
        
        self.confidence_threshold = 0.25
        self.frequency = 1/50
        self.rgb_image = None
        self.depth_image = None
        
        # Subscriber for RGB image
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.rgb_image_callback,
            10)
        
        # Subscriber for depth image
        self.depth_subscription = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_image_callback,
            10)
        
        # Publisher for segmented image
        self.image_publisher = self.create_publisher(
            Image,
            '/probe_detector/segmented_image',
            10)
                   
        self.bridge = CvBridge()
        
        # Load YOLO model
        package_name = 'probe_detection'
        try:
            package_share_dir = get_package_share_directory(package_name)
            self.model_path = os.path.join(package_share_dir, 'models', 'probe_segmentation.pt')
            if not os.path.exists(self.model_path):
                raise FileNotFoundError(f"Model file not found at: {self.model_path}")
            self.model = YOLO(self.model_path)
            self.get_logger().info(f"Loaded YOLO model from {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize model: {e}")
            raise
        
        #
        self.handle_image_timer = self.create_timer(self.frequency, self.handle_images)

    def rgb_image_callback(self, msg):
        """Callback for RGB image messages."""
        try:
            # Convert ROS Image to OpenCV format
            if msg.encoding == "bgra8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgra8")
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            self.rgb_image = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert RGB image: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in RGB callback: {e}")

    def depth_image_callback(self, msg):
        """Callback for depth image messages."""
        try:
            # Convert ROS depth image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            self.depth_image = depth_image
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in depth callback: {e}")

    def image_handler(self):
    # check current image pair, if valid process further, or else discard
    # to be valid a time check must also be made
    
    # rotate both the image and the depth: 
    rgb_image = cv2.rotate(rgb_image, cv2.ROTATE_180)
    depth_image = cv2.rotate(depth_image, cv2.ROTATE_180)
    
    # Infer yolo on image, and find probes
    #probes = infer_yolo()
    
    # Calculate probe localization
    # if (probes):
        # probe_locations =  self.compute_probe_location(probes, depth_image)  

    def handle_images(self):
        """Process RGB and depth images when both are available."""
        if self.rgb_image is None or self.depth_image is None:
            self.get_logger().info("Waiting for both RGB and depth images...")
            return

        # Rotate images
        rgb_image = cv2.rotate(self.rgb_image, cv2.ROTATE_180)
        depth_image = cv2.rotate(self.depth_image, cv2.ROTATE_180)

        # Run YOLO inference
        probe_boxes, probe_masks, probe_confidences = self.infer_yolo(rgb_image)
 
        # Calculate probe localization if detections exist
        if probe_boxes:
            self.get_logger().info("Probes detected, computing locations...")
            # Uncomment and implement when ready
            # probe_locations = self.compute_probe_location(probe_boxes, depth_image)
            print("hello")  # Temporary debug output

    def compute_probe_location(self, probes, depth_image):
        """
        Compute 3D locations of probes using depth data.
        Placeholder - implement as needed.
        """
        # Add your depth-based localization logic here
        pass

    def infer_yolo(self, rgb_image: np.ndarray) -> Tuple[List[List[float]], List[List[float]], List[float]]:
        """
        Run YOLO inference on an RGB image and return detection data.
        
        Args:
            rgb_image (np.ndarray): Input RGB image for inference
            
        Returns:
            Tuple containing:
                - probe_boxes: List of bounding box coordinates [x_min, y_min, x_max, y_max]
                - probe_masks: List of flattened mask data
                - probe_confidences: List of confidence scores
        """
        probe_boxes = []
        probe_masks = []
        probe_confidences = []

        try:
            # Run YOLO inference
            results = self.model.predict(
                source=rgb_image,
                imgsz=416,
                conf=self.confidence_threshold,
                device=0 
            )
            
            # Publish segmented image
            segmented_image = results[0].plot()
            try:
                segmented_msg = self.bridge.cv2_to_imgmsg(segmented_image, "bgr8")
                self.image_publisher.publish(segmented_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"Failed to publish segmented image: {e}")

            # Process detection data
            if results[0].boxes is not None and results[0].masks is not None:
                for i, (box, mask) in enumerate(zip(results[0].boxes, results[0].masks)):
                    try:
                        # Convert mask data to list
                        mask_data = mask.data[0].cpu().numpy()
                        mask_list = mask_data.flatten().tolist()
                        
                        # Store detection data
                        probe_boxes.append([float(coord) for coord in box.xyxy[0]])
                        probe_masks.append(mask_list)
                        probe_confidences.append(float(box.conf))
                        
                    except Exception as e:
                        self.get_logger().error(f"Error processing detection {i}: {e}")
                        continue

        except Exception as e:
            self.get_logger().error(f"Error in YOLO inference: {e}")

        return probe_boxes, probe_masks, probe_confidences

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SegmentationNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()