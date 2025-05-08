#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, PointCloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Empty
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import struct

class CaptureNode(Node):
    def __init__(self):
        super().__init__('capture_node')
        
        # Output directory for saving files
        self.output_dir = "src/probe_detection/test"
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        # Counter for file naming
        self.capture_count = 1
        
        # Flag to control capture
        self.capture_triggered = False
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Subscribers for synchronized data
        self.rgb_sub = Subscriber(self, CompressedImage, '/zed/zed_node/rgb/image_rect_color/compressed')
        self.depth_sub = Subscriber(self, PointCloud2, '/zed/zed_node/point_cloud/cloud_registered')
        
        # Approximate Time Synchronizer
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_handler)
        
        # Subscriber for capture trigger
        self.trigger_sub = self.create_subscription(
            Empty,
            '/capture_trigger',
            self.on_trigger,
            10
        )
        
        self.get_logger().info("Node started. Publish to /capture_trigger to capture a frame.")
        self.get_logger().info("To take a picture, run: ros2 topic pub --once /capture_trigger std_msgs/msg/Empty '{}'")

    def on_trigger(self, msg):
        """Callback for capture trigger."""
        self.capture_triggered = True
        self.get_logger().info("Capture triggered.")
        
    def image_handler(self, rgb_msg, depth_msg):
        """Handles synchronized RGB and point cloud data."""
        if not self.capture_triggered:
            return  # Wait for trigger
        
        try:
            # Generate file name
            file_name = f"image_{self.capture_count:03d}"
            
            # Convert RGB image
            np_arr = np.frombuffer(rgb_msg.data, np.uint8)
            rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Convert point cloud to array
            point_cloud = self.pointcloud2_to_array(depth_msg)
            
            # Extract depth (z-values) from point cloud
            depth_image = point_cloud[:, :, 2]  # Z-channel
            
            # Save RGB image
            rgb_path = os.path.join(self.output_dir, f"{file_name}.png")
            cv2.imwrite(rgb_path, rgb_image)
            self.get_logger().info(f"Saved RGB image to {rgb_path}")
            
            # Save point cloud
            pc_path = os.path.join(self.output_dir, f"{file_name}.npy")
            np.save(pc_path, point_cloud)
            self.get_logger().info(f"Saved point cloud to {pc_path}")
            
            # Save depth image
            depth_path = os.path.join(self.output_dir, f"{file_name}_depth.npy")
            np.save(depth_path, depth_image)
            self.get_logger().info(f"Saved depth image to {depth_path}")
            
            # Increment counter and reset trigger
            self.capture_count += 1
            self.capture_triggered = False
            
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to process image: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in image_handler: {e}")
            
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
        if msg.height == 1:
            self.get_logger().warn('Point cloud is unstructured (height=1), treating as 1D array')
        
        n_points = msg.height * msg.width
        data = msg.data
        
        expected_size = msg.row_step * msg.height
        if len(data) != expected_size:
            self.get_logger().error(f'Data size mismatch: expected {expected_size}, got {len(data)}')
            return None
        
        height, width = msg.height, msg.width
        pointcloud_array = np.zeros((height, width, 3), dtype=np.float32)
        
        for i in range(n_points):
            offset = i * msg.point_step
            x = struct.unpack_from('<f', data, offset + 0)[0]
            y = struct.unpack_from('<f', data, offset + 4)[0]
            z = struct.unpack_from('<f', data, offset + 8)[0]
            
            row = i // msg.width
            col = i % msg.width
            pointcloud_array[row, col, 0] = x
            pointcloud_array[row, col, 1] = y
            pointcloud_array[row, col, 2] = z
        
        return pointcloud_array

def main(args=None):
    rclpy.init(args=args)
    node = None  # Initialize node to None
    try:
        node = CaptureNode()
        rclpy.spin(node)

        
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()