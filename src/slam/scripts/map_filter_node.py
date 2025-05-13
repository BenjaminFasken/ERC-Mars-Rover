import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Quaternion
import math # Added import

# Function to convert quaternion to yaw angle (radians) - Added function
def quaternion_to_yaw(q: Quaternion) -> float:
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class MapFilterNode(Node):
    def __init__(self):
        super().__init__('map_filter_node')
        # Subscribers
        self.map_sub = self.create_subscription(
            # OccupancyGrid, '/map', self.map_callback, 10)
            OccupancyGrid, '/global_costmap/costmap', self.map_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/traversed_path', self.path_callback, 10)
            # Path, '/zed/zed_node/path_map', self.path_callback, 10)
        # Publisher
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/filtered_map', 10)
        # Storage
        self.latest_map = None
        self.latest_path = None

    def map_callback(self, msg):
        """Store the latest traversability map."""
        self.latest_map = msg
        self.get_logger().info('Received new map')

    def path_callback(self, msg):
        """Process the path and generate a filtered map."""
        self.latest_path = msg
        if self.latest_map is None:
            self.get_logger().warn('No map received yet')
            return
        self.generate_filtered_map()

    def generate_filtered_map(self):
        """Create a new map based on the path and publish it."""
        map_msg = self.latest_map
        path_msg = self.latest_path

        # Map properties
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        # Convert path poses to map coordinates
        points = []
        for pose_stamped in path_msg.poses: # Renamed variable for clarity
            # x = pose_stamped.pose.position.x + 2.032197628484078 # TODO: Remove hardcoded offset?
            # y = pose_stamped.pose.position.y + 0.5129117160870764 # TODO: Remove hardcoded offset?
            x = pose_stamped.pose.position.x # TODO: Remove hardcoded offset?
            y = pose_stamped.pose.position.y  # TODO: Remove hardcoded offset?
            col = int((x - origin_x) / resolution)
            row = int((y - origin_y) / resolution)
            # Ensure points are within map bounds
            if 0 <= col < width and 0 <= row < height:
                points.append((col, row))
            else:
                self.get_logger().warn(f"Path point ({col}, {row}) is outside map bounds ({width}, {height}). Skipping.")


        # Create a blank mask
        mask = np.zeros((height, width), dtype=np.uint8)

        # Draw the path on the mask
        if len(points) == 0:
            # Empty path: mask remains all zeros
            pass
        else:
            # Multiple points: draw lines between consecutive poses
            thickness = max(1, int(2 / resolution))  # At least 1 cell thick, maybe wider
            # Use polylines for potentially better performance and handling of connections
            # pts_np = np.array(points, dtype=np.int32)
            # cv2.polylines(mask, [pts_np], isClosed=False, color=255, thickness=thickness)
            # Also draw cones (sectors) at each point to ensure coverage and indicate direction
            cone_radius = max(1, thickness//2) # Adjust radius as needed
            cone_angle_width_deg = 100.0 # Width of the cone in degrees

            for i, pt in enumerate(points):
                # Calculate direction angle
                if i < len(points) - 1:
                    # Direction from current point to next point
                    dx = points[i+1][0] - pt[0]
                    dy = points[i+1][1] - pt[1]
                elif len(points) > 1:
                    # Direction from previous point to current point (for the last point)
                    dx = pt[0] - points[i-1][0]
                    dy = pt[1] - points[i-1][1]
                else:
                    # Should not happen if len(points) > 1, but handle defensively
                    dx, dy = 1, 0 # Default direction (e.g., right)

                # Angle in radians (math standard: 0=right, positive=CCW)
                # Note: OpenCV y-axis points down, so dy is inverted for angle calculation
                angle_rad = math.atan2(-dy, dx)
                # Convert to OpenCV angle (degrees, 0=right, positive=CW)
                angle_deg = -math.degrees(angle_rad)

                # Calculate start and end angles for the cone sector
                start_angle = angle_deg - cone_angle_width_deg / 2.0
                end_angle = angle_deg + cone_angle_width_deg / 2.0

                # Draw the filled sector (cone)
                cv2.ellipse(mask, pt, axes=(cone_radius, cone_radius), angle=0,
                            startAngle=start_angle, endAngle=end_angle,
                            color=255, thickness=-1) # thickness=-1 fills the sector


        # Reshape original map data to 2D
        original_data = np.array(map_msg.data, dtype=np.int8).reshape((height, width))

        # Create new map data: keep original where mask > 0, else -1
        # Ensure we handle the -1 (unknown) case in the original map correctly
        # If the original cell is unknown (-1), keep it unknown even if it's on the path mask
        new_data_2d = np.full_like(original_data, -1) # Start with all unknown
        mask_indices = mask > 0
        new_data_2d[mask_indices] = original_data[mask_indices]
        # Apply threshold: values < 90 (and not -1) are set to 0.
        # This is to clear low-confidence obstacles from the path.
        # Values >= 90 (definite obstacles) and -1 (unknown) are preserved.
        threshold_val = 45
        # Create a boolean mask for elements that are less than threshold_val AND not -1
        condition_for_zeroing = (new_data_2d < threshold_val) & (new_data_2d != -1)
        # Apply the zeroing to elements meeting the condition
        new_data_2d[condition_for_zeroing] = 0

        # Prepare the new map message
        new_map = OccupancyGrid()
        new_map.header = map_msg.header
        # Update timestamp to reflect when the filtering happened
        new_map.header.stamp = self.get_clock().now().to_msg()
        new_map.info = map_msg.info
        new_map.data = new_data_2d.flatten().tolist()

        # Publish the filtered map
        self.map_pub.publish(new_map)
        self.get_logger().info('Published filtered map')

def main(args=None):
    rclpy.init(args=args)
    node = MapFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()