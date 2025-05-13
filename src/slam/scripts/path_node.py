#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from std_msgs.msg import Header
import math


# Function to convert quaternion to yaw angle (radians)
def quaternion_to_yaw(q: Quaternion) -> float:
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# Function to calculate the shortest angle difference (radians)
def angle_diff(a1, a2):
    diff = a1 - a2
    while diff < -math.pi:
        diff += 2.0 * math.pi
    while diff > math.pi:
        diff -= 2.0 * math.pi
    return abs(diff)


class PathNode(Node):
    def __init__(self):
        super().__init__('path_node')
        self.subscription = self.create_subscription(
            PoseStamped,  # Changed from Odometry
            '/zed/zed_node/pose', # Changed topic
            self.pose_callback,  # Renamed callback for clarity
            10)
        self.publisher_ = self.create_publisher(
            Path,
            '/my_path',
            10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom' # Initialize frame_id
        self.last_publish_time = self.get_clock().now()
        self.last_appended_pose: Pose | None = None
        self.min_dist_sq = 0.05 * 0.05
        self.min_angle_rad = math.radians(12.0)
        # Add parameter for max path length
        self.declare_parameter('max_path_poses', 500) # Store approx last 500 poses
        self.max_path_poses = self.get_parameter('max_path_poses').get_parameter_value().integer_value
        self.get_logger().info(f"Path node initialized. Max path poses: {self.max_path_poses}")


    def pose_callback(self, msg: PoseStamped): # Renamed and type hint changed
        # Ensure frame_id is consistent
        if not self.path_msg.header.frame_id:
             self.path_msg.header.frame_id = msg.header.frame_id
        elif self.path_msg.header.frame_id != msg.header.frame_id:
             self.get_logger().warn(f"Pose frame_id changed from {self.path_msg.header.frame_id} to {msg.header.frame_id}. Resetting path.")
             self.path_msg.poses = []
             self.last_appended_pose = None
             self.path_msg.header.frame_id = msg.header.frame_id


        current_pose = msg.pose # Changed: PoseStamped directly contains Pose
        should_append = False

        if self.last_appended_pose is None:
            should_append = True
        else:
            # Calculate position change
            dx = current_pose.position.x - self.last_appended_pose.position.x
            dy = current_pose.position.y - self.last_appended_pose.position.y
            dist_sq = dx*dx + dy*dy

            # Calculate orientation change
            current_yaw = quaternion_to_yaw(current_pose.orientation)
            last_yaw = quaternion_to_yaw(self.last_appended_pose.orientation)
            delta_yaw = angle_diff(current_yaw, last_yaw)

            # Check thresholds
            if dist_sq > self.min_dist_sq or delta_yaw > self.min_angle_rad:
                should_append = True

        if should_append:
            # PoseStamped message is already in the correct format to be added to Path
            self.path_msg.poses.append(msg) # Changed: append the whole PoseStamped msg
            self.last_appended_pose = current_pose

            # --- Limit the number of poses in the path ---
            if len(self.path_msg.poses) > self.max_path_poses:
                # Remove the oldest poses to maintain max length
                num_to_remove = len(self.path_msg.poses) - self.max_path_poses
                self.path_msg.poses = self.path_msg.poses[num_to_remove:]
                # self.get_logger().debug(f"Path trimmed to {self.max_path_poses} poses.") # Optional debug log


        # --- Publishing logic ---
        current_time = self.get_clock().now()
        if current_time - self.last_publish_time >= Duration(seconds=1):
            if self.path_msg.poses:
                # Update path header timestamp to the latest pose timestamp
                self.path_msg.header.stamp = self.path_msg.poses[-1].header.stamp
            else:
                # If no poses, use current time
                self.path_msg.header.stamp = current_time.to_msg()

            self.publisher_.publish(self.path_msg)
            self.last_publish_time = current_time


def main(args=None):
    rclpy.init(args=args)
    path_node = PathNode()
    try:
        rclpy.spin(path_node)
    except KeyboardInterrupt:
        pass
    path_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
