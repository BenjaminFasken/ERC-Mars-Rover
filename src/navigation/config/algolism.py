import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point, Quaternion  # Added Quaternion
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool  # Added for manual triggering
from nav_msgs.msg import Odometry
import numpy as np
import random
import math  # Added math

class FrontierExplorationNode(Node):
    def __init__(self):
        super().__init__('frontier_exploration_node')
        # Subscriptions
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, 'filtered_map', self.map_callback, 10)
        self.goal_status_subscriber = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',  # Correct topic for NavigateToPose
            self.goal_status_callback,
            10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/zed/zed_node/odom', self.odom_callback, 10)
        self.trigger_subscriber = self.create_subscription(
            Bool,
            '/trigger_exploration',  # New topic for manual triggering
            self.trigger_callback,
            10)
        
        # Publishers
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        
        # Internal state
        self.homing = False
        self.map_data = None
        self.map_array = None
        self.map_metadata = None
        self.goal_reached = False  # Initialize to True to allow first goal
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.map_info = None
        self.timer = self.create_timer(5, self.timer_callback)

    def goal_status_callback(self, msg):
        if msg.status_list:
            if not self.homing:
                current_status = msg.status_list[-1].status
                if current_status == 4:  # STATUS_SUCCEEDED
                    self.goal_reached = True
                    self.get_logger().info("Goal succeeded")
                elif current_status == 6:  # STATUS_ABORTED
                    self.goal_reached = True
                    self.get_logger().info("Goal aborted")
                elif current_status == 2:  # STATUS_EXECUTING
                    self.goal_reached = False
                    self.get_logger().info("Goal is executing")
                else:
                    self.goal_reached = False
                    self.get_logger().info(f"Goal status: {current_status}")

    def trigger_callback(self, msg):
        if msg.data:
            self.homing = False
            self.goal_reached = True
            self.get_logger().info("Exploration manually triggered")
        if not msg.data:
            self.goal_reached = False
            self.homing = True
            self.get_logger().info("Exploration stopped, going HOME")
            zero_orientation_q = Quaternion()
            zero_orientation_q.x = 0.0
            zero_orientation_q.y = 0.0
            zero_orientation_q.z = 0.0
            zero_orientation_q.w = 1.0  # Neutral orientation
            self.publish_goal([0.0, 0.0], zero_orientation_q)

    def map_callback(self, msg):
        self.map_data = msg.data
        self.map_array = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_metadata = msg.info
        self.map_info = msg.info

    def timer_callback(self):
        if self.map_array is None:
            self.get_logger().warn("Map not yet received, skipping")
            return
        if self.goal_reached:
            self.get_logger().info("Detecting new frontier")
            frontiers = self.detect_frontiers(self.map_info)
            if frontiers:
                goal_xy = self.select_goal(frontiers, self.map_array)  # Renamed for clarity
                if goal_xy[0] is not None:
                    # Calculate orientation towards the goal
                    orientation_q = self.calculate_orientation_to_goal(goal_xy[0], goal_xy[1])
                    self.publish_goal(goal_xy, orientation_q)
                else:
                    self.get_logger().warn("No valid frontiers found")
            else:
                self.get_logger().warn("No frontiers detected")
        else:
            self.get_logger().info("Waiting for current goal to complete")

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def is_near_wall(self, x, y, map_data, threshold):
        for i in range(-threshold, threshold + 1):
            for j in range(-threshold, threshold + 1):
                check_x = x + i
                check_y = y + j
                if 0 <= check_x < map_data.shape[1] and 0 <= check_y < map_data.shape[0]:
                    if map_data[check_y, check_x] == 100:
                        return True
        return False

    def detect_frontiers(self, map_info):
        frontiers = []
        for y in range(map_info.height):
            for x in range(map_info.width):
                if self.map_array[y, x] == -1:  # Unknown area
                    neighbors = self.map_array[max(0, y-1):min(map_info.height, y+2),
                                              max(0, x-1):min(map_info.width, x+2)]
                    if 0 in neighbors:  # Free space nearby
                        frontiers.append((x, y))
        return frontiers

    def select_goal(self, frontiers, map_data):
        valid_frontiers = [f for f in frontiers if not self.is_near_wall(f[0], f[1], map_data, threshold=3)]
        if valid_frontiers:
            goal_x, goal_y = random.choice(valid_frontiers)
            real_x = (goal_x + 0.5) * self.map_metadata.resolution + self.map_metadata.origin.position.x
            real_y = (goal_y + 0.5) * self.map_metadata.resolution + self.map_metadata.origin.position.y
            return real_x, real_y
        return None, None

    def calculate_orientation_to_goal(self, goal_x, goal_y):
        """
        Calculates the orientation (as a Quaternion) from the robot's current pose
        to the given goal coordinates.
        """
        # Calculate the difference in coordinates
        delta_x = goal_x - self.robot_x
        delta_y = goal_y - self.robot_y
        
        # Calculate the yaw angle
        yaw = math.atan2(delta_y, delta_x)
        # Convert yaw to quaternion
        # For 2D navigation, roll and pitch are 0
        orientation_q = Quaternion()
        orientation_q.x = 0.0
        orientation_q.y = 0.0
        orientation_q.z = math.sin(yaw / 2.0)
        orientation_q.w = math.cos(yaw / 2.0)
        
        return orientation_q

    def publish_goal(self, goal_xy, orientation_q):  # Modified signature to include orientation
        if goal_xy[0] is None or goal_xy[1] is None:
            self.get_logger().warn("No valid frontier found, skipping goal publication")
            return
        
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"  # Ensure this is your fixed frame
        
        # Set the position
        goal_msg.pose.position.x = goal_xy[0]
        goal_msg.pose.position.y = goal_xy[1]
        goal_msg.pose.position.z = 0.0  # Assuming 2D navigation
        
        # Set the orientation
        goal_msg.pose.orientation = orientation_q
        
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published goal: x={goal_xy[0]:.2f}, y={goal_xy[1]:.2f}, "
                               f"orientation_z={orientation_q.z:.2f}, orientation_w={orientation_q.w:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()