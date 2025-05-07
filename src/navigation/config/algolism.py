import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool  # Added for manual triggering
from nav_msgs.msg import Odometry
import numpy as np
import random

class FrontierExplorationNode(Node):
    def __init__(self):
        super().__init__('frontier_exploration_node')
        # Subscriptions
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
        self.goal_status_subscriber = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',  # Correct topic for NavigateToPose
            self.goal_status_callback,
            10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.trigger_subscriber = self.create_subscription(
            Bool,
            '/trigger_exploration',  # New topic for manual triggering
            self.trigger_callback,
            10)
        
        # Publishers
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        
        # Internal state
        self.map_data = None
        self.map_array = None
        self.map_metadata = None
        self.goal_reached = True  # Initialize to True to allow first goal
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.map_info = None
        self.timer = self.create_timer(5, self.timer_callback)

    def goal_status_callback(self, msg):
        if msg.status_list:
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
            self.goal_reached = True
            self.get_logger().info("Exploration manually triggered")

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
                goal = self.select_goal(frontiers, self.map_array)
                if goal[0] is not None:
                    self.publish_goal(goal)
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

    def publish_goal(self, goal):
        if goal[0] is None or goal[1] is None:
            self.get_logger().warn("No valid frontier found, skipping goal publication")
            return
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position = Point(x=goal[0], y=goal[1], z=0.0)
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published goal: x={goal[0]}, y={goal[1]}")

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()