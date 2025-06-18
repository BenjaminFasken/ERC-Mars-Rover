#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/probe_locations.hpp"
#include "interfaces/msg/probe_data.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp> 
#include <cmath>
#include <deque>
#include <mutex>

using ProbeLocations = interfaces::msg::ProbeLocations;
using ProbeData = interfaces::msg::ProbeData;

// Probe class
class Probe {
public:
  Probe(float x, float y, float z, float confidence)
      : x_sum_(x), y_sum_(y), z_sum_(z), confidence_sum_(confidence), count_(1) {}

  void update(float x, float y, float z, float confidence) {
    x_sum_ += x;
    y_sum_ += y;
    z_sum_ += z;
    confidence_sum_ += confidence;
    count_++;
  }

  float distanceTo(float x, float y, float z) const {
    auto [avg_x, avg_y, avg_z] = getAveragePosition();
    float dx = avg_x - x;
    float dy = avg_y - y;
    //float dz = avg_z - z;
    //return std::sqrt(dx * dx + dy * dy + dz * dz);
    return std::sqrt(dx * dx + dy * dy);
  }

  std::tuple<float, float, float> getAveragePosition() const {
    return {x_sum_ / count_, y_sum_ / count_, z_sum_ / count_};
  }

  float getAverageConfidence() const { return confidence_sum_ / count_; }

  int getCount() const { return count_; }

private:
  float x_sum_;
  float y_sum_;
  float z_sum_;
  float confidence_sum_;
  int count_;
};


// Node class
class ProbeFilteringNode : public rclcpp::Node {
public:
  ProbeFilteringNode() : Node("probe_filtering_node") {
    // Subscribe to vision-system detections
    subscription_ = this->create_subscription<ProbeLocations>(
        "/probe_detector/probe_locations", 10,
        std::bind(&ProbeFilteringNode::probeCallback, this, std::placeholders::_1));

    // Subscribe to global pose
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/zed/zed_node/pose", 10,
        std::bind(&ProbeFilteringNode::poseCallback, this, std::placeholders::_1));

    // Existing publisher for filtered probes
    publisher_ = this->create_publisher<ProbeData>("/filtered_probes", 10);

    // New publisher for RViz visualization
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "probe_markers", 10);

    RCLCPP_INFO(this->get_logger(), "Probe filtering Node initialized.");
  }

private:
  bool pose_received_ = false;

  // Return a copy of the pose history
  std::vector<geometry_msgs::msg::PoseStamped> getPoseHistoryCopy() const {
    std::lock_guard<std::mutex> lock(pose_mutex_);

    if (pose_history_.empty()) {
      throw std::runtime_error("No poses in history");
    }

    return std::vector<geometry_msgs::msg::PoseStamped>(pose_history_.begin(), pose_history_.end());
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    latest_pose_ = *msg;
    pose_received_ = true;

    // Maintain fixed size history of 30 poses
    if (pose_history_.size() >= 30) {
      pose_history_.pop_front();  // Remove oldest
    }

    // Modify the msg 

    pose_history_.push_back(*msg);  // Add newest

    RCLCPP_DEBUG(get_logger(), "Received new localization_pose");
  }

  // Finds the pose closest to the given timestamp
  geometry_msgs::msg::PoseStamped getClosestPose(const rclcpp::Time& query_time) const {

    // safe copy of pose history
    std::vector<geometry_msgs::msg::PoseStamped> pose_history_copy = getPoseHistoryCopy();

    auto closest = pose_history_copy.front();
    rclcpp::Duration min_diff = absTimeDiff(query_time, closest.header.stamp);

    for (const auto& pose : pose_history_copy) {
      auto diff = absTimeDiff(query_time, pose.header.stamp);
      if (diff < min_diff) {
        closest = pose;
        min_diff = diff;
      }
    }
    // print time difference
    RCLCPP_INFO(get_logger(), "Time difference: %f", min_diff.seconds());
    return closest;
  }
  
  // Helper to compute absolute time difference
  rclcpp::Duration absTimeDiff(const rclcpp::Time& t1, const rclcpp::Time& t2) const {
    return (t1 > t2) ? (t1 - t2) : (t2 - t1);
  }

  std::tuple<float, float, float> transformToGlobal(float lx, float ly, float lz, const geometry_msgs::msg::PoseStamped &matched_pose) {
    if (!pose_received_) {
      RCLCPP_WARN(get_logger(),
                  "No global pose yet; returning camera coords unchanged");
      return {lx, ly, lz};
    }

    // 1) First build the point in your camera frame, then into the rover (base_link) frame
    tf2::Matrix3x3 R_cam2base(0.9397, 0.0,  -0.3420,
                              0.0,    1.0,   0.0,
                              0.3420, 0.0,   0.9397);
    tf2::Vector3 t_cam2base(-0.146422, -0.0598990, -0.238857);
    tf2::Transform T_cam2base(R_cam2base, t_cam2base);

    tf2::Vector3 p_cam(lx, ly, lz);
    tf2::Vector3 p_base = T_cam2base * p_cam;

    // 2) Now build the transform from base_link → global using the provided pose
    const auto &pos = matched_pose.pose.position;
    const auto &ori = matched_pose.pose.orientation;
    tf2::Transform T_base2world;
    T_base2world.setOrigin(tf2::Vector3(pos.x, pos.y, pos.z));
    tf2::Quaternion q;
    tf2::fromMsg(ori, q);
    T_base2world.setRotation(q);

    // 3) Apply that full transform
    tf2::Vector3 p_world = T_base2world * p_base;

    return {
      static_cast<float>(p_world.x()),
      static_cast<float>(p_world.y()),
      static_cast<float>(p_world.z())
    };
  }


  // New function to publish probes as RViz markers
  void publishProbeMarkers() {

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = latest_pose_.header.frame_id;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "probes_sphere_list"; // Namespace for the marker
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST; // Changed to SPHERE_LIST for multiple points
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set marker properties
    marker.scale.x = 0.1; // Diameter of each sphere (10 cm)
    marker.scale.y = 0.1;
    marker.scale.z = 0.1; // Uniform scale for spheres
    marker.color.r = 0.0; // Base color (transparent to rely on per-point colors)
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.0; // Transparent base color

    // Set pose (required for SPHERE_LIST, typically at origin if points are in global frame)
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0; // No rotation
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;

    // Add each probe as a point with color based on confidence, only if seen more than once
    for (const auto &p : tracked_probes_) {
        if (p.getCount() < 2) continue; // Only visualize probes seen more than once

        auto [x, y, z] = p.getAveragePosition();
        float confidence = p.getAverageConfidence();

        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = 0; //z; //<- brug z hvis den skal plottes i 3D
        marker.points.push_back(point);

        // Color based on confidence thresholds
        std_msgs::msg::ColorRGBA color;
        if (confidence < 0.7) {
            // Below 70%: Red
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
        } else if (confidence <= 0.8) {
            // 70% to 80%: Yellow
            color.r = 1.0;
            color.g = 1.0;
            color.b = 0.0;
        } else if (confidence > 0.9) {
            // Above 90%: Green
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
        } else {
            // 80% to 90%: Greenish-yellow
            color.r = 0.5;
            color.g = 1.0;
            color.b = 0.0;
        }
        color.a = 1.0;
        marker.colors.push_back(color);
    }

    // Publish the marker
    marker_publisher_->publish(marker);
  } 

  void publishProbeModels() {
    // Directory where the mesh file is stored (e.g., a .stl or .dae file referenced by the URDF)
    const std::string mesh_resource_path = "package://navigation/assets/probeModel.dae";

    int marker_id = 0; // Unique ID for each marker

    // Iterate over tracked probes
    for (const auto &p : tracked_probes_) {
        if (p.getCount() < 2) continue; // Only visualize probes seen more than once

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = latest_pose_.header.frame_id;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "probes_models";
        marker.id = marker_id++; // Assign unique ID for each probe
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE; // Use MESH_RESOURCE for URDF/mesh
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the mesh resource
        marker.mesh_resource = mesh_resource_path;
        marker.mesh_use_embedded_materials = true; // Use materials defined in the mesh (if any)

        // Set marker scale (adjust based on your mesh's size)
        marker.scale.x = 1.0; // Scale factor for the mesh
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Set pose based on probe's average position
        auto [x, y, z] = p.getAveragePosition();
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z+10.0; // Use z for 3D positioning
        marker.pose.orientation.w = 1.0; // No rotation (adjust if needed)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;

        // Set color based on confidence
        float confidence = p.getAverageConfidence();
        std_msgs::msg::ColorRGBA color;
        if (confidence < 0.7) {
            // Below 70%: Red
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
        } else if (confidence <= 0.8) {
            // 70% to 80%: Yellow
            color.r = 1.0;
            color.g = 1.0;
            color.b = 0.0;
        } else if (confidence > 0.9) {
            // Above 90%: Green
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
        } else {
            // 80% to 90%: Greenish-yellow
            color.r = 0.5;
            color.g = 1.0;
            color.b = 0.0;
        }
        color.a = 1.0;
        marker.color = color;

        // Publish the marker
        marker_publisher_->publish(marker);
    }
}
  
  void probeCallback(const ProbeLocations::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received new probe locations");

    // find the closest pose to the probe message timestamp
    auto closest_pose = getClosestPose(msg->header.stamp);

    bool new_probe_found = false;
    for (size_t i = 0; i < msg->num_probes; ++i) {
      size_t base = i * 3;
      float lx = msg->probes[base + 0];
      float ly = msg->probes[base + 1];
      float lz = msg->probes[base + 2];
      float confidence = msg->classification_confidence[i];
      float distance = std::sqrt(lx * lx + ly * ly + lz * lz);

      if (distance > 2.5) {
        RCLCPP_WARN(get_logger(), "Probe too far away: %f", distance);
        continue;
      }

      auto [gx, gy, gz] = transformToGlobal(lx, ly, lz, closest_pose);

      gz = 0.0; // Set z to 0 for 2D visualization

      Probe incoming_probe(gx, gy, gz, confidence);
      bool matched_existing = false;
      for (Probe &tracked : tracked_probes_) {
        RCLCPP_INFO(get_logger(), "Incoming probe: %f %f %f", gx, gy, gz);
        auto [tx, ty, tz] = tracked.getAveragePosition();
        // RCLCPP_INFO(get_logger(), "Tracked probe: %f %f %f", tx, ty, tz);
        // RCLCPP_INFO(get_logger(), "Distance to existing probe: %f", tracked.distanceTo(gx, gy, gz));

        if (tracked.distanceTo(gx, gy, gz) < merge_threshold_) {
          tracked.update(gx, gy, gz, confidence);
          matched_existing = true;
          break;
        }
      }
      if (!matched_existing) {
        tracked_probes_.push_back(incoming_probe);
        new_probe_found = true;
      }
    }

    // ProbeData publishing
    ProbeData msg_out;
    msg_out.stamp = this->get_clock()->now();

    // Only include probes with contribution >= 2
    size_t valid_probe_count = 0;
    for (auto &p : tracked_probes_) {
      if (p.getCount() >= 2) {
        auto [ax, ay, az] = p.getAveragePosition();
        msg_out.x.push_back(ax);
        msg_out.y.push_back(ay);
        msg_out.z.push_back(az);
        msg_out.confidence.push_back(p.getAverageConfidence());
        msg_out.contribution.push_back(p.getCount());
        ++valid_probe_count;
      }
    }
    msg_out.probe_count = valid_probe_count;
    publisher_->publish(msg_out);

    // Add marker publishing for RViz
    publishProbeMarkers();
    publishProbeModels();
  }

  float merge_threshold_ = 0.6; // distance in meters to merge probes
  std::vector<Probe> tracked_probes_;
  rclcpp::Subscription<ProbeLocations>::SharedPtr subscription_;
  rclcpp::Publisher<ProbeData>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_; // New marker publisher
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  std::deque<geometry_msgs::msg::PoseStamped> pose_history_;
  mutable std::mutex pose_mutex_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProbeFilteringNode>());
  rclcpp::shutdown();
  return 0;
}