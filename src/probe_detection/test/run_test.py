#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from interfaces.msg import ProbeLocations
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
from pathlib import Path
import csv
import time
from scipy.spatial import distance_matrix
from scipy.optimize import linear_sum_assignment

class FillAnnotationNode(Node):
    def __init__(self):
        super().__init__('fill_annotation_node')
        self.bridge = CvBridge()

        # Publishers
        self.rgb_pub = self.create_publisher(
            CompressedImage, '/zed/zed_node/rgb/image_rect_color/compressed', 10)
        self.depth_pub = self.create_publisher(
            PointCloud2, '/zed/zed_node/point_cloud/cloud_registered', 10)

        # Subscriber
        self.probe_sub = self.create_subscription(
            ProbeLocations, '/probe_detector/probe_locations',
            self.probe_callback, 10)

        # Directories
        self.base_dir = Path("~/ERC-Mars-Rover/src/probe_detection/test").expanduser()
        self.data_dir = self.base_dir / "data"
        self.annotation_dir = self.base_dir / "annotations"
        self.output_dir = self.base_dir / "Annotated_filledOut"
        self.output_dir.mkdir(exist_ok=True)

        # Load transformation matrix
        transform_path = self.data_dir / "transformation_matrix.npy"
        if not transform_path.exists():
            self.get_logger().error(f"Transformation matrix not found at {transform_path}")
            raise FileNotFoundError(f"Missing {transform_path}")
        self.transform_matrix = np.load(transform_path)

        # Load input files
        self.image_files = sorted(self.data_dir.glob("image_*.png"))
        self.pointcloud_files = sorted(f for f in self.data_dir.glob("image_*.npy") if "depth" not in f.stem)
        self.annotation_files = sorted(self.annotation_dir.glob("image_*_annotation.json"))

        # Validate file counts
        if len(self.image_files) != 10 or len(self.pointcloud_files) != 10 or len(self.annotation_files) != 10:
            self.get_logger().error(f"Missing input files. Expected 10 images, point clouds, and annotations.")
            self.get_logger().error(f"Found {len(self.image_files)} images, {len(self.pointcloud_files)} point clouds, {len(self.annotation_files)} annotations.")
            raise FileNotFoundError("Incorrect number of input files")

        # Store received probe locations
        self.received_probes = {}
        self.current_image_id = None

        # Results for error computation
        self.results = []

        # Image dimensions
        self.image_width = 640
        self.image_height = 360

    def pointcloud_to_msg(self, pointcloud, image_id):
        """Convert numpy point cloud to PointCloud2 message."""
        height, width, _ = pointcloud.shape
        if height != self.image_height or width != self.image_width:
            self.get_logger().error(f"Point cloud shape {pointcloud.shape} does not match expected ({self.image_height}, {self.image_width}, 3)")
            return None

        points = pointcloud.reshape(-1, 3)
        points = points[~np.isnan(points).any(axis=1)]  # Remove invalid points

        # Create PointCloud2 message (structured)
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.height = height
        msg.width = width
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.point_step = 12  # 3 floats (x, y, z) * 4 bytes
        msg.row_step = msg.point_step * width
        msg.data = pointcloud.astype(np.float32).tobytes()
        msg.is_bigendian = False
        msg.is_dense = False  # Allow NaN points

        return msg

    def probe_callback(self, msg):
        """Callback for receiving probe locations."""
        if self.current_image_id is None:
            return

        # Extract probe locations
        num_probes = msg.num_probes
        probes = []
        probe_coords = msg.probes
        confidences = msg.classification_confidence
        centroids_x = msg.centroid_x
        centroids_y = msg.centroid_y

        for i in range(num_probes):
            x = probe_coords[i * 3]
            y = probe_coords[i * 3 + 1]
            z = probe_coords[i * 3 + 2]
            confidence = confidences[i]
            centroid_x = centroids_x[i]
            centroid_y = centroids_y[i]
            probes.append({
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "confidence": float(confidence),
                "centroid_x": float(centroid_x),
                "centroid_y": float(centroid_y)
            })

        self.received_probes[self.current_image_id] = probes
        self.get_logger().info(f"Received {num_probes} probe locations for {self.current_image_id}: {probes}")

    def visualize_image(self, image_path, annotation_path, image_id, matches):
        """Visualize GT and detected probes for a single image."""
        img = cv2.imread(str(image_path))
        if img is None:
            self.get_logger().error(f"Failed to read image: {image_path}")
            return

        # Load annotation
        with open(annotation_path, 'r') as f:
            annotation = json.load(f)

        # Get detected probes
        detected_probes = self.received_probes.get(image_id, [])

        # Draw GT probes
        for gt_idx, probe_data in enumerate(annotation["probes"]):
            bbox = probe_data["bounding_box"]
            center_x = int(bbox[0] * self.image_width)
            center_y = int(bbox[1] * self.image_height)
            cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)  # Red for GT
            cv2.putText(img, f"GT{gt_idx}", (center_x + 10, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Draw detected probes with matched GT indices
        for det_idx, probe in enumerate(detected_probes):
            centroid_x, centroid_y = int(probe["centroid_x"]), int(probe["centroid_y"])
            cv2.circle(img, (centroid_x, centroid_y), 5, (0, 255, 0), -1)  # Green for detected
            # Find matched GT index, if any
            matched_gt_idx = None
            for gt_idx, d_idx in matches:
                if d_idx == det_idx:
                    matched_gt_idx = gt_idx
                    break
            label = f"Det{det_idx}:GT{matched_gt_idx}" if matched_gt_idx is not None else f"Det{det_idx}"
            cv2.putText(img, label, (centroid_x + 10, centroid_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # Show the image
        cv2.imshow(f"Probes - {image_id}", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        self.get_logger().info(f"Showing image with probes for {image_id}")

    def run_test(self):
        """Process each image and point cloud pair, publish, update annotations, and visualize."""
        for img_path, pc_path, ann_path in zip(self.image_files, self.pointcloud_files, self.annotation_files):
            image_id = img_path.stem  # e.g., image_001
            self.current_image_id = image_id

            # Read and publish RGB image
            img = cv2.imread(str(img_path))
            if img is None:
                self.get_logger().error(f"Failed to read image: {img_path}")
                continue
            compressed_img = self.bridge.cv2_to_compressed_imgmsg(img)
            compressed_img.header.stamp = self.get_clock().now().to_msg()
            compressed_img.header.frame_id = "camera_link"
            self.rgb_pub.publish(compressed_img)

            # Read and publish point cloud
            pointcloud = np.load(pc_path)
            if pointcloud.shape[-1] != 3:
                self.get_logger().error(f"Invalid point cloud shape for {pc_path}: {pointcloud.shape}")
                continue
            pointcloud_msg = self.pointcloud_to_msg(pointcloud, image_id)
            if pointcloud_msg is None:
                self.get_logger().error(f"Failed to create PointCloud2 message for {pc_path}")
                continue
            self.depth_pub.publish(pointcloud_msg)

            # Wait for probe locations (timeout 10 seconds)
            start_time = time.time()
            while image_id not in self.received_probes and time.time() - start_time < 10:
                rclpy.spin_once(self, timeout_sec=0.1)

            if image_id not in self.received_probes:
                self.get_logger().warn(f"No probe locations received for {image_id}")
                continue

            # Update and save annotation
            matches = self.update_annotation(image_id, ann_path)
            self.get_logger().info(f"Updated annotation for {image_id}")

            # Visualize the processed image
            #self.visualize_image(img_path, ann_path, image_id, matches)

        # Save results to CSV
        self.save_results()

    def update_annotation(self, image_id, ann_path):
        """Update annotation file with estimated and transformed coordinates."""
        # Load existing annotation for the input image
        with open(ann_path, 'r') as f:
            annotation = json.load(f)

        # Get received probes for the input image
        detected_probes = self.received_probes.get(image_id, [])

        # Extract ground truth bounding box centers
        gt_centers = []
        for probe_data in annotation["probes"]:
            bbox = probe_data["bounding_box"]  # [center_x, center_y, width, height], normalized
            center_x = bbox[0] * self.image_width  # Convert to pixels
            center_y = bbox[1] * self.image_height
            gt_centers.append([center_x, center_y])

        # Extract detected probe mask centers
        det_centers = [[p["centroid_x"], p["centroid_y"]] for p in detected_probes]

        # Match detected probes to ground truth using Hungarian algorithm
        matches = []
        if gt_centers and det_centers:
            dist_matrix = distance_matrix(gt_centers, det_centers)
            row_ind, col_ind = linear_sum_assignment(dist_matrix)
            for gt_idx, det_idx in zip(row_ind, col_ind):
                distance = dist_matrix[gt_idx, det_idx]
                if distance < 50:  # Threshold in pixels (adjust based on image resolution)
                    matches.append((gt_idx, det_idx))
                else:
                    self.get_logger().warn(f"No close match for GT probe {gt_idx + 1} in {image_id}, min distance: {distance}")

        # Log unmatched GT probes
        matched_gt_indices = set(gt_idx for gt_idx, _ in matches)
        for gt_idx in range(len(gt_centers)):
            if gt_idx not in matched_gt_indices:
                self.get_logger().warn(f"No match found for GT probe {gt_idx + 1} in {image_id}")

        # Update probes with matched estimated and transformed coordinates for the input image
        for gt_idx, probe_data in enumerate(annotation["probes"]):
            matched = False
            for gt_idx_match, det_idx in matches:
                if gt_idx == gt_idx_match:
                    est = detected_probes[det_idx]
                    probe_data["estimated_coordinates_camera"] = {
                        "x": est["x"],
                        "y": est["y"],
                        "z": est["z"]
                    }

                    # Apply transformation
                    est_vec = np.array([est["x"], est["y"], est["z"], 1])
                    trans_vec = np.linalg.inv(self.transform_matrix) @ est_vec
                    probe_data["transformed_coordinates_ground_plane"] = {
                        "x": float(trans_vec[0]),
                        "y": float(trans_vec[1]),
                        "z": float(trans_vec[2])
                    }

                    # Compute error (convert ground truth from cm to meters)
                    gt = probe_data["ground_truth_coordinates_ground_plane"]
                    gt_m = {"x": gt["x"] / 100, "y": gt["y"] / 100, "z": gt["z"] / 100}
                    trans = probe_data["transformed_coordinates_ground_plane"]
                    error = np.sqrt(
                        (trans["x"] - gt_m["x"])**2 +
                        (trans["y"] - gt_m["y"])**2 +
                        (trans["z"] - gt_m["z"])**2
                    )

                    self.results.append({
                        "image_id": image_id,
                        "probe_id": probe_data["identifier"],
                        "error": error,
                        "x_estimated": est["x"],
                        "y_estimated": est["y"],
                        "z_estimated": est["z"],
                        "x_transformed": trans["x"],
                        "y_transformed": trans["y"],
                        "z_transformed": trans["z"],
                        "x_gt": gt_m["x"],
                        "y_gt": gt_m["y"],
                        "z_gt": gt_m["z"],
                        "centroid_x": est["centroid_x"],
                        "centroid_y": est["centroid_y"]
                    })
                    matched = True
                    break
            if not matched:
                self.get_logger().warn(f"No match found for GT probe {probe_data['identifier']} in {image_id}")

        # Save updated annotation for the input image
        output_path = self.output_dir / f"{image_id}_annotation.json"
        with open(output_path, "w") as f:
            json.dump(annotation, f, indent=4)

        return matches  # Return matches for visualization

    def save_results(self):
        """Save results to CSV using csv module."""
        output_path = self.output_dir / "results.csv"
        headers = [
            "image_id", "probe_id", "error",
            "x_estimated", "y_estimated", "z_estimated",
            "x_transformed", "y_transformed", "z_transformed",
            "x_gt", "y_gt", "z_gt",
            "centroid_x", "centroid_y"
        ]

        with open(output_path, "w", newline='') as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()
            for result in self.results:
                writer.writerow(result)
        self.get_logger().info(f"Results saved to {output_path}")

def main(args=None):
    rclpy.init(args=args)
    node = FillAnnotationNode()
    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()