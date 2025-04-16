#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import struct
import binascii
from std_msgs.msg import Bool, Float32, UInt8
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from interfaces.msg import ProbeLocations
import os
from cv_bridge import CvBridge
import cv2

# Determine the base directory of the project (ERC-Mars-Rover root)
workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))
pictures_dir = os.path.join(workspace_dir, 'src', 'gcs', 'gcs_gui', 'pictures')
# Ensure the directory exists
os.makedirs(pictures_dir, exist_ok=True)

max_voltage = 21.0
min_voltage = 15.0

class RoverTelemetryNode(Node):
    def __init__(self):
        super().__init__('rover_telemetry')
        self.declare_parameter('tm_host', '127.0.0.1')
        self.declare_parameter('tm_port', 10015)
        self.tm_host = self.get_parameter('tm_host').value
        self.tm_port = self.get_parameter('tm_port').value
        self.tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tm_counter = 0
        self.state = {
            'connection_status': False,
            'rover_position': Vector3(x=0.0, y=0.0, z=0.0),
            'euler_rotations': Vector3(x=0.0, y=0.0, z=0.0),
            'velocity': Vector3(x=0.0, y=0.0, z=0.0),
            'probe_positions': [(0.0, 0.0, 0.0)] * 3,
            'probe_confidences': [0.0] * 3,
            'image_update': False,
            'battery_voltage': 0.0,
            'Battery_Charge': 0.0,
            'Battery_change': False,
            'mode': 0
        }
        self.sub_status = self.create_subscription(Bool, '/rover/status', self.status_callback, 10)
        self.sub_position = self.create_subscription(Vector3, '/rover/position', self.position_callback, 10)
        self.sub_attitude = self.create_subscription(Vector3, '/rover/attitude', self.attitude_callback, 10)
        self.sub_velocity = self.create_subscription(Vector3, '/rover/velocity', self.velocity_callback, 10)
        self.sub_probes = self.create_subscription(ProbeLocations, '/rover/probes', self.probes_callback, 10)
        #self.sub_image = self.create_subscription(Image, '/rover/image', self.image_callback, 10)
        self.sub_battery = self.create_subscription(Float32, '/rover/battery', self.battery_callback, 10)
        self.sub_mode = self.create_subscription(UInt8, '/rover/mode', self.mode_callback, 10)
        self.create_timer(0.5, self.print_status)
        self.get_logger().info(f"Started RoverTelemetry: TM host={self.tm_host}, TM port={self.tm_port}")

    def status_callback(self, msg):
        self.state['connection_status'] = msg.data
        self.send_telemetry()

    def position_callback(self, msg):
        self.state['rover_position'] = msg
        self.send_telemetry()

    def attitude_callback(self, msg):
        self.state['euler_rotations'] = msg
        self.send_telemetry()

    def velocity_callback(self, msg):
        self.state['velocity'] = msg
        self.send_telemetry()

    def probes_callback(self, msg):
        probes = []
        confidences = msg.classification_confidence[:3]
        for i in range(0, min(msg.num_probes * 3, len(msg.probes), 9), 3):
            probes.append((msg.probes[i], msg.probes[i+1], msg.probes[i+2]))
        while len(probes) < 3:
            probes.append((0.0, 0.0, 0.0))
            confidences.append(0.0)
        self.state['probe_positions'] = probes[:3]
        self.state['probe_confidences'] = confidences[:3]
        self.send_telemetry()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

            # Save the image to the pictures directory as PNG
            image_path = os.path.join(pictures_dir, 'image.png')
            cv2.imwrite(image_path, cv_image)
            self.get_logger().info(f"Saved image to {image_path}")

            # Set the image update flag to True
            self.state['image_update'] = True
        except Exception as e:
            self.get_logger().error(f"Error processing and saving image: {str(e)}")
            self.state['image_update'] = False

        # Send telemetry after processing the image
        self.send_telemetry()
        
        # Reset the image update flag, such that i does not get sent in the next telemetry packet unless a new image is received
        self.state['image_update'] = False

    def battery_callback(self, msg):
        self.state['battery_voltage'] = msg.data
        self.state['Battery_Charge'] = (msg.data - min_voltage) / (max_voltage - min_voltage) * 100
        self.state['Battery_change'] = self.state['Battery_Charge'] > 15
        self.send_telemetry()

    def mode_callback(self, msg):
        self.state['mode'] = msg.data
        self.send_telemetry()

    def send_telemetry(self):
        try:
            self.get_logger().info(
                f"Sending: status={self.state['connection_status']}, "
                f"pos=({self.state['rover_position'].x:.2f}, {self.state['rover_position'].y:.2f}, {self.state['rover_position'].z:.2f}), "
                f"probes={[f'({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})' for p in self.state['probe_positions']]}, "
                f"confidences={[f'{c:.2f}' for c in self.state['probe_confidences']]}, "
                f"image_update={self.state['image_update']}, "
                f"Battery_Charge={self.state['Battery_Charge']:.2f}, Battery_change={self.state['Battery_change']}, "
                f"voltage={self.state['battery_voltage']:.2f}, mode={self.state['mode']}"
            )
            packet = struct.pack(
                '!?'   # Connection_Status (bool, 1 byte)
                '3f'   # Rover_Position (3 floats, 12 bytes)
                '3f'   # Euler_Rotations (3 floats, 12 bytes)
                '3f'   # Velocity (3 floats, 12 bytes)
                '9f'   # Probe_Positions (9 floats, 36 bytes)
                '3f'   # Probe_Confidences (3 floats, 12 bytes)
                '?'    # Image_Update (bool, 1 byte)
                'f'    # Battery_Voltage (1 float, 4 bytes)
                'f'    # Battery_Charge (1 float, 4 bytes)
                '?'    # Battery_change (bool, 1 byte)
                'B',   # Mode (1 byte)
                self.state['connection_status'],
                float(self.state['rover_position'].x), float(self.state['rover_position'].y), float(self.state['rover_position'].z),
                float(self.state['euler_rotations'].x), float(self.state['euler_rotations'].y), float(self.state['euler_rotations'].z),
                float(self.state['velocity'].x), float(self.state['velocity'].y), float(self.state['velocity'].z),
                self.state['probe_positions'][0][0], self.state['probe_positions'][0][1], self.state['probe_positions'][0][2],
                self.state['probe_positions'][1][0], self.state['probe_positions'][1][1], self.state['probe_positions'][1][2],
                self.state['probe_positions'][2][0], self.state['probe_positions'][2][1], self.state['probe_positions'][2][2],
                self.state['probe_confidences'][0], self.state['probe_confidences'][1], self.state['probe_confidences'][2],
                self.state['image_update'],
                self.state['battery_voltage'],
                self.state['Battery_Charge'],
                self.state['Battery_change'],
                self.state['mode']
            )
            self.tm_socket.sendto(packet, (self.tm_host, self.tm_port))
            self.tm_counter += 1
            self.get_logger().info(f"Packed packet: {binascii.hexlify(packet[:64]).decode()}... ({len(packet)} bytes)")
            # Reset image_update flag after sending
            self.state['image_update'] = False
        except Exception as e:
            self.get_logger().error(f"Error packing/sending telemetry: {str(e)}")

    def print_status(self):
        self.get_logger().info(f"Sent {self.tm_counter} telemetry packets")

    def destroy_node(self):
        self.get_logger().info("Closing UDP socket")
        self.tm_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = RoverTelemetryNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()