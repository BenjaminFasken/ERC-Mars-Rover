#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
import csv
import os
from datetime import datetime
import math
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

def euler_from_quaternion(quaternion):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw)
    Input is in the form [x, y, z, w]
    """
    x, y, z, w = quaternion
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

class OdometryComparisonNode(Node):
    def __init__(self):
        super().__init__('odometry_comparison_node')
        
        # Create CSV file with timestamp in name
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"odometry_comparison_{timestamp}.csv"
        self.csv_path = os.path.join(os.getcwd(), self.csv_filename)
        
        # Setup CSV file
        self.csvfile = open(self.csv_path, 'w', newline='')
        self.fieldnames = [
            'timestamp', 
            'zed_pos_x', 'zed_pos_y', 'zed_pos_z',
            'zed_roll', 'zed_pitch', 'zed_yaw',
            'vehicle_pos_x', 'vehicle_pos_y', 'vehicle_pos_z', 
            'vehicle_heading',
            'time_diff'
        ]
        self.csv_writer = csv.DictWriter(self.csvfile, fieldnames=self.fieldnames)
        self.csv_writer.writeheader()
        
        # Create QoS profile for better reliability
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create subscribers
        self.zed_sub = self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.zed_callback,
            qos
        )
        
        self.vehicle_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_callback,
            qos
        )
        
        # Data storage
        self.latest_zed = None
        self.latest_zed_time = 0.0
        self.latest_vehicle = None
        self.latest_vehicle_time = 0.0
        
        # Buffer for batched writing to CSV
        self.data_buffer = []
        self.buffer_size = 10
        
        # Counter for logs
        self.counter = 0
        self.log_frequency = 100  # Log every 100 callbacks
        
        # Timer for matching and writing data
        self.timer = self.create_timer(0.05, self.match_and_save)  # 20 Hz
        
        self.get_logger().info(f"Created CSV file at {self.csv_path}")
    
    def zed_callback(self, msg):
        """Process ZED odometry messages"""
        self.latest_zed = msg
        self.latest_zed_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Log occasionally to confirm we're getting data
        self.counter += 1
        if self.counter % self.log_frequency == 0:
            self.get_logger().info(f"Received ZED data, timestamp: {self.latest_zed_time}")
    
    def vehicle_callback(self, msg):
        """Process vehicle local position messages"""
        self.latest_vehicle = msg
        # PX4 timestamp is in microseconds
        self.latest_vehicle_time = msg.timestamp * 1e-6  # Convert microseconds to seconds
        
        # Log occasionally to confirm we're getting data
        if self.counter % self.log_frequency == 0:
            self.get_logger().info(f"Received vehicle data, timestamp: {self.latest_vehicle_time}")
    
    def match_and_save(self):
        """Match messages by timestamp and save to CSV"""
        if not self.latest_zed or not self.latest_vehicle:
            return
        
        # Check if timestamps are close enough (within 100ms)
        time_diff = abs(self.latest_zed_time - self.latest_vehicle_time)
        if time_diff > 0.1:
            if self.counter % self.log_frequency == 0:
                self.get_logger().warn(f"Timestamp mismatch: ZED {self.latest_zed_time}, Vehicle {self.latest_vehicle_time}, diff: {time_diff}")
            return
            
        try:
            # Extract ZED pose data
            zed_pos_x = self.latest_zed.pose.pose.position.x
            zed_pos_y = self.latest_zed.pose.pose.position.y
            zed_pos_z = self.latest_zed.pose.pose.position.z
            
            # Convert quaternion to Euler angles
            quat = [
                self.latest_zed.pose.pose.orientation.x,
                self.latest_zed.pose.pose.orientation.y,
                self.latest_zed.pose.pose.orientation.z,
                self.latest_zed.pose.pose.orientation.w
            ]
            # Use the corrected euler_from_quaternion function
            zed_roll, zed_pitch, zed_yaw = euler_from_quaternion(quat)
            
            # Extract vehicle local position data from PX4 message
            # Check if the position is valid before using
            vehicle_pos_x = self.latest_vehicle.x #if self.latest_vehicle.xy_valid else float('nan')
            vehicle_pos_y = self.latest_vehicle.y #if self.latest_vehicle.xy_valid else float('nan')
            vehicle_pos_z = self.latest_vehicle.z #if self.latest_vehicle.z_valid else float('nan')
            
            # Get heading (yaw) - note that PX4 doesn't directly provide roll and pitch
            vehicle_heading = self.latest_vehicle.heading
            
            # Create data point
            data_point = {
                'timestamp': self.latest_zed_time,
                'zed_pos_x': zed_pos_x,
                'zed_pos_y': zed_pos_y,
                'zed_pos_z': zed_pos_z,
                'zed_roll': zed_roll,
                'zed_pitch': zed_pitch,
                'zed_yaw': zed_yaw,
                'vehicle_pos_x': vehicle_pos_x,
                'vehicle_pos_y': vehicle_pos_y,
                'vehicle_pos_z': vehicle_pos_z,
                'vehicle_heading': vehicle_heading,
                'time_diff': time_diff
            }
            
            # Add to buffer
            self.data_buffer.append(data_point)
            
            # Write to CSV if buffer is full
            if len(self.data_buffer) >= self.buffer_size:
                for point in self.data_buffer:
                    self.csv_writer.writerow(point)
                self.csvfile.flush()  # Ensure data is written to disk
                self.data_buffer = []
                self.get_logger().info(f"Wrote {self.buffer_size} data points to CSV")
                
        except Exception as e:
            self.get_logger().error(f"Error processing messages: {str(e)}")
    
    def __del__(self):
        """Clean up resources"""
        if hasattr(self, 'csvfile'):
            # Write any remaining data
            if hasattr(self, 'data_buffer') and self.data_buffer:
                for point in self.data_buffer:
                    self.csv_writer.writerow(point)
            self.csvfile.close()
            self.get_logger().info(f"Closed CSV file: {self.csv_path}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryComparisonNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()