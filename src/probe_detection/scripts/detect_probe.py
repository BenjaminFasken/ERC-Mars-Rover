#!/usr/bin/env python3
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, PointCloud2
from interfaces.msg import ProbeSegmentation, ProbeLocations 
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from typing import Tuple, List
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import struct


class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')
        
        self.previous_time = 0
        self.fps = 1
        self.confidence_threshold = 0.75
        self.rgb_image = None
        self.depth_image = None
        
        # Inside SegmentationNode.__init__()
        self.rgb_sub = Subscriber(self, CompressedImage, '/zed/zed_node/rgb/image_rect_color/compressed')    
        self.depth_sub = Subscriber(self, PointCloud2, '/zed/zed_node/point_cloud/cloud_registered')

        # Approximate Time Synchronizer
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_handler)    
        
        # Publisher for segmented image
        self.image_publisher = self.create_publisher(
            Image,
            '/probe_detector/segmented_image',
            10)
        
        self.probe_publisher = self.create_publisher(
            ProbeLocations,
            '/probe_detector/probe_locations',
            10
        )
                   
        self.bridge = CvBridge()
        
        # Load YOLO model
        package_name = 'probe_detection'
        try:
            package_share_dir = get_package_share_directory(package_name)
            self.model_path = os.path.join(package_share_dir, 'models', 'YOLO11s.pt')
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

            # skip frames if the time difference is too small
            if self.previous_time + (1 / self.fps) > rgb_time:
                return
            else:
                self.previous_time = rgb_time

            self.get_logger().info(f"Synchronized RGB Time: {rgb_time}, Depth Time: {depth_time}")

            # Convert RGB image
            np_arr = np.frombuffer(rgb_msg.data, np.uint8)
            rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Convert pointcloud2
            point_cloud = self.pointcloud2_to_array(depth_msg)
            
            # Run YOLO inference
            probe_boxes, probe_masks, probe_confidences = self.infer_yolo(rgb_image)
            
            # If probes are detected, process them
            if probe_masks:
                # Compute locations with confidences
                probe_locations = self.compute_probe_locations(probe_masks, point_cloud, rgb_image, probe_confidences)

                if probe_locations:
                    sorted_locations = sorted(probe_locations, key=lambda x: x["confidence"], reverse=True)

                    # Debugging output
                    print(f"Probe Locations: {sorted_locations}")
                    
                    probe_confidences = [float(np.float32(loc["confidence"])) for loc in sorted_locations]
                    probe_list = [float(np.float32(coord)) for loc in sorted_locations for coord in (loc["x"], loc["y"], loc["z"])]
                    probe_centroid_x = [float(np.float32(loc["centroid_x"])) for loc in sorted_locations]
                    probe_centroid_y = [float(np.float32(loc["centroid_y"])) for loc in sorted_locations]
       
                    # Create and publish ProbeLocations message
                    probe_msg = ProbeLocations()
                    probe_msg.header = rgb_msg.header
                    probe_msg.classification_confidence = probe_confidences
                    probe_msg.num_probes = len(sorted_locations)
                    probe_msg.probes = probe_list
                    probe_msg.centroid_x = probe_centroid_x
                    probe_msg.centroid_y = probe_centroid_y

                    # Publish the message (assuming a publisher exists)
                    self.probe_publisher.publish(probe_msg)
                    self.get_logger().info(f"Published {probe_msg.num_probes} probe locations")

        
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to process image: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in image_handler: {e}")

    def compute_probe_locations(self, probe_masks: List[np.ndarray], point_cloud: np.ndarray, rgb_image: np.ndarray, probe_confidences: List[float]) -> List[dict]:
        """Compute 3D locations of probes using depth data."""
        probe_locations = []
        
        # print rgb image size and depth image size
        self.get_logger().info(f"RGB Image Size: {rgb_image.shape}")
        self.get_logger().info(f"Depth Image Size: {point_cloud.shape}")
        
        # extract depth image from point cloud
        x_image = point_cloud[:, :, 0]  # Depth channel
        y_image = point_cloud[:, :, 1]
        z_image = point_cloud[:, :, 2]
        
        for i, mask in enumerate(probe_masks):
            try:
                position = []    
                # resize mask to match the depth image size
                mask = cv2.resize(mask, (x_image.shape[1], x_image.shape[0]), interpolation=cv2.INTER_LINEAR)
                        
                # Normalize mask to binary (0 or 1)
                binary_mask = (mask > 0.5).astype(np.uint8)
                                                                    
                # compute the center of the mask
                y, x = np.where(binary_mask == 1)
                if x.size > 0 and y.size > 0:
                    centroid_x = int(np.mean(x))
                    centroid_y = int(np.mean(y))
                    
                    # Extract a 3x3 kernel around the centroid
                    kernel_size = 3
                    half_kernel = kernel_size // 2
                    x_min = max(centroid_x - half_kernel, 0)
                    x_max = min(centroid_x + half_kernel + 1, x_image.shape[1])
                    y_min = max(centroid_y - half_kernel, 0)
                    y_max = min(centroid_y + half_kernel + 1, x_image.shape[0])
                    kernel = binary_mask[y_min:y_max, x_min:x_max]
                    kernel = kernel.astype(np.uint8)
                    
                    position_kernel = np.array([x_image[y_min:y_max, x_min:x_max], y_image[y_min:y_max, x_min:x_max], z_image[y_min:y_max, x_min:x_max]])
                    
                    #calculate the meadian of the kernel, excluding NaN and Inf values
                    valid_x = position_kernel[0][kernel == 1]
                    valid_y = position_kernel[1][kernel == 1]
                    valid_z = position_kernel[2][kernel == 1]
                
                    valid_x = valid_x[np.isfinite(valid_x)]
                    valid_y = valid_y[np.isfinite(valid_y)]
                    valid_z = valid_z[np.isfinite(valid_z)]

                    median_x = np.nanmedian(valid_x)
                    median_y = np.nanmedian(valid_y)
                    median_z = np.nanmedian(valid_z)
                    
                    #Position is the median of the kernel
                    position = np.array([median_x, median_y, median_z])

                    # Check if the position is valid (not NaN or Inf)
                    if np.isnan(position).any() or np.isinf(position).any():
                        continue
                else:
                    #jump out of current forloop iteration, and continue with the next mask
                    continue  
                           
                probe_location = {
                    "centroid_x": centroid_x,
                    "centroid_y": centroid_y,
                    "confidence": probe_confidences[i],
                    "x": position[0],
                    "y": position[1],
                    "z": position[2]
                }
                probe_locations.append(probe_location)
                  
            except Exception as e:
                self.get_logger().error(f"Error processing mask {i}: {e}")
                continue



        # publish the depth values next to the probe in the rgb image
        resized_rgb = cv2.resize(rgb_image, (x_image.shape[1], x_image.shape[0]), interpolation=cv2.INTER_LINEAR)
        for loc in probe_locations:
            centroid_x = loc["centroid_x"]
            centroid_y = loc["centroid_y"]
            position = [loc["x"], loc["y"], loc["z"]]
            confidence = loc["confidence"]
            cv2.circle(resized_rgb, (centroid_x, centroid_y), 3, (0, 0, 255), -1)  # Red dot
            cv2.putText(resized_rgb, f"conf={confidence:.2f}, x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}", 
                (centroid_x + 5, centroid_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    
        try:
            segmented_msg = self.bridge.cv2_to_imgmsg(resized_rgb, "bgr8")
            self.image_publisher.publish(segmented_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to publish segmented image: {e}")


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
            
            # Publish to ros2 the segmented image
            # annotated_image = results[0].plot()
            # try:
            #     segmented_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
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
    
    def pointcloud2_to_array(self, msg):
        """
        Convert a sensor_msgs/PointCloud2 message to a 3D array of x, y, z coordinates.
        
        Args:
            msg: sensor_msgs.msg.PointCloud2 message
            
        Returns:
            pointcloud_array: np.array of shape (height, width, 3) where:
                - First dimension: height (rows, vertical axis)
                - Second dimension: width (columns, horizontal axis)
                - Third dimension: [x, y, z] coordinates
        """
        # Check if the message is organized
        if msg.height == 1:
            self.get_logger().warn('Point cloud is unstructured (height=1), treating as 1D array')
        
        # Total number of points
        n_points = msg.height * msg.width
        
        # Extract raw data
        data = msg.data  # Byte array
        
        # Verify data length matches expected size
        expected_size = msg.row_step * msg.height
        if len(data) != expected_size:
            self.get_logger().error(f'Data size mismatch: expected {expected_size}, got {len(data)}')
            return None

        # Initialize 3D array for point cloud
        height, width = msg.height, msg.width
        pointcloud_array = np.zeros((height, width, 3), dtype=np.float32)  # Shape: (height, width, 3)

        # Parse the data
        for i in range(n_points):
            # Calculate byte offset for this point
            offset = i * msg.point_step
            
            # Extract x, y, z (float32, little-endian)
            x = struct.unpack_from('<f', data, offset + 0)[0]
            y = struct.unpack_from('<f', data, offset + 4)[0]
            z = struct.unpack_from('<f', data, offset + 8)[0]
            
            # Map to (row, col) in the array
            row = i // msg.width  # Row index (height dimension, vertical axis)
            col = i % msg.width   # Column index (width dimension, horizontal axis)
            
            # Store x, y, z in the third dimension
            pointcloud_array[row, col, 0] = x
            pointcloud_array[row, col, 1] = y
            pointcloud_array[row, col, 2] = z

            # Coordinate system is defined as per ros2 convention. X is forward, Y is left, Z is up. Defined on the left lens

        return pointcloud_array


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