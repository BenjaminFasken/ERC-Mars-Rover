import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np
import threading


class MapPathVisualizer(Node):
    def __init__(self):
        super().__init__('map_path_visualizer')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)

        self.map_data = None
        self.map_info = None
        self.path_poses = []

        self.display_thread = threading.Thread(target=self.display_loop)
        self.display_thread.daemon = True
        self.display_thread.start()

    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))
        # Convert occupancy values to grayscale (0=free, 100=occupied, -1=unknown)
        self.map_data = np.where(data == -1, 205, 255 - data).astype(np.uint8)

    def path_callback(self, msg: Path):
        self.path_poses = msg.poses

    def display_loop(self):
        plt.ion()
        fig, ax = plt.subplots()
        while rclpy.ok():
            if self.map_data is not None and self.path_poses and self.map_info:
                ax.clear()
                ax.imshow(self.map_data, cmap='gray', origin='lower')

                res = self.map_info.resolution
                origin = self.map_info.origin.position

                xs = []
                ys = []
                for pose_stamped in self.path_poses:
                    x = (pose_stamped.pose.position.x - origin.x) / res
                    y = (pose_stamped.pose.position.y - origin.y) / res
                    xs.append(x)
                    ys.append(y)

                ax.plot(xs, ys, color='red', linewidth=2)
                ax.set_title("Path on Map")
                plt.draw()
                plt.pause(0.1)
            else:
                plt.pause(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = MapPathVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
