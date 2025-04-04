#!/usr/bin/env python3
from message_filters import ApproximateTimeSynchronizer, Subscriber
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
        self.rgb_image = None
        self.depth_image = None
        
        # Inside SegmentationNode.__init__()
        self.rgb_sub = Subscriber(self, Image, '/zed/zed_node/rgb/image_rect_color')    
        self.depth_sub = Subscriber(self, Image, '/zed/zed_node/depth/depth_registered')

        # Approximate Time Synchronizer
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_handler)    
        
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

    def image_handler(self, rgb_msg, depth_msg):
        """Handles synchronized RGB and depth images."""
        try:
            # Extract timestamps for debugging
            rgb_time = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
            depth_time = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9
            self.get_logger().info(f"Synchronized RGB Time: {rgb_time}, Depth Time: {depth_time}")

            # Convert RGB Image
            if rgb_msg.encoding == "bgra8":
                rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgra8")
                rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGRA2BGR)
            else:
                rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")

            # Convert Depth Image
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")

            # Run YOLO inference
            probe_boxes, probe_masks, probe_confidences = self.infer_yolo(rgb_image)

            # If probes are detected, process them
            if probe_masks:
                probe_locations = self.compute_probe_location(probe_masks, depth_image, rgb_image)
                self.get_logger().info(f"Probe locations: {probe_locations}")
        
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to process image: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in image_handler: {e}")

    def compute_probe_location(self, probe_masks: List[np.ndarray], depth_image: np.ndarray, rgb_image: np.ndarray) -> List[dict]:
        """Compute 3D locations of probes using depth data."""
        sample_radius = 4
        probe_locations = []

        for i, mask in enumerate(probe_masks):
            try:             
                # Normalize mask to binary (0 or 1)
                binary_mask = (mask > 0.5).astype(np.uint8)
                                                            
                # compute the center of the mask
                y, x = np.where(binary_mask == 1)
                if x.size > 0 and y.size > 0:
                    centroid_x = int(np.mean(x))
                    centroid_y = int(np.mean(y))
                else:
                    #jump out of current forloop iteration, and continue with the next mask
                    continue  
                
                # Sample depth values around the centroid    
                x_min = max(centroid_x - sample_radius, 0)
                x_max = min(centroid_x + sample_radius, depth_image.shape[1])
                y_min = max(centroid_y - sample_radius, 0)
                y_max = min(centroid_y + sample_radius, depth_image.shape[0])

                # place a small red dot on the centroid and publish
                # cv2.circle(rgb_image, (centroid_x, centroid_y), 2, (0, 0, 255), -1)
                # try:
                #     segmented_msg = self.bridge.cv2_to_imgmsg(rgb_image, "bgr8")
                #     self.image_publisher.publish(segmented_msg)
                # except CvBridgeError as e:
                #     self.get_logger().error(f"Failed to publish segmented image: {e}")

                # Extract the sampled depth values
                sampled_depth_values = depth_image[y_min:y_max, x_min:x_max].flatten()

                # Filter out NaN and infinite values
                sampled_depth_values = sampled_depth_values[np.isfinite(sampled_depth_values)]
                if sampled_depth_values.size > 0:
                    # Compute median depth, it acts as a low pass filter
                    median_depth = np.median(sampled_depth_values)   
                else:
                    # not valid
                    continue
                     
                probe_location = {
                    "centroid_x": centroid_x,
                    "centroid_y": centroid_y,
                    "mean_depth": float(median_depth)
                }
                probe_locations.append(probe_location)
               
                # publish the depth values next to the probe in the rgb image
                cv2.putText(rgb_image, f"{median_depth:.2f}", (centroid_x, centroid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                try:
                    segmented_msg = self.bridge.cv2_to_imgmsg(rgb_image, "bgr8")
                    self.image_publisher.publish(segmented_msg)
                except CvBridgeError as e:
                    self.get_logger().error(f"Failed to publish segmented image: {e}")
          
               
            except Exception as e:
                self.get_logger().error(f"Error processing mask {i}: {e}")
                continue

        return probe_locations

    def infer_yolo(self, rgb_image: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray], np.ndarray]:
        probe_boxes = []
        probe_masks = []
        probe_confidences = []

        try:
            results = self.model.predict(
                source=rgb_image,
                imgsz=640,
                conf=self.confidence_threshold,
                device=0 
            )
            
            segmented_image = results[0].plot()
            # try:
            #     segmented_msg = self.bridge.cv2_to_imgmsg(segmented_image, "bgr8")
            #     self.image_publisher.publish(segmented_msg)
            # except CvBridgeError as e:
            #     self.get_logger().error(f"Failed to publish segmented image: {e}")

            if results[0].boxes is not None and results[0].masks is not None:
                for i, (box, mask) in enumerate(zip(results[0].boxes, results[0].masks)):
                    try:
                        box_xyxy = box.xyxy[0].cpu().numpy()
                        mask_data = mask.data[0].cpu().numpy()

                        if rgb_image is not None:
                            mask_data = cv2.resize(mask_data, (rgb_image.shape[1], rgb_image.shape[0]))

                        probe_boxes.append(box_xyxy)
                        probe_masks.append(mask_data)
                        probe_confidences.append(float(box.conf.cpu()))
                        
                    except Exception as e:
                        self.get_logger().error(f"Error processing detection {i}: {e}")
                        continue

            probe_boxes = np.array(probe_boxes, dtype=np.float32) if probe_boxes else np.array([], dtype=np.float32).reshape(0, 4)
            probe_confidences = np.array(probe_confidences, dtype=np.float32) if probe_confidences else np.array([], dtype=np.float32)

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