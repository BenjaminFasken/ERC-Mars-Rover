#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/probe_locations.hpp"
#include "interfaces/msg/probe_data.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp> 
#include <cmath>

using ProbeLocations = interfaces::msg::ProbeLocations;
using ProbeData = interfaces::msg::ProbeData;

// [Probe class unchanged]
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
    float dz = avg_z - z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
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
    publisher_ = this->create_publisher<ProbeData>("filtered_probes", 10);

    // New publisher for RViz visualization
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "probe_markers", 10);

    RCLCPP_INFO(this->get_logger(), "Probe filtering Node initialized.");
  }

private:
  bool pose_received_ = false;

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    latest_pose_ = *msg;
    pose_received_ = true;
    RCLCPP_DEBUG(get_logger(), "Received new localization_pose");
  }

  std::tuple<float, float, float> transformToGlobal(float lx, float ly, float lz) {
    if (!pose_received_) {
      RCLCPP_WARN(get_logger(),
                  "No global pose yet; returning camera coords unchanged");
      return {lx, ly, lz};
    }

    tf2::Matrix3x3 R_cam2base(0.9397, 0.0, -0.3420, 0.0, 1.0, 0.0, 0.3420, 0.0,
                              0.9397);
    tf2::Vector3 t_cam2base(-0.146422, -0.0598990, -0.238857);
    tf2::Transform T_cam2base(R_cam2base, t_cam2base);

    tf2::Vector3 p_cam(lx, ly, lz);
    tf2::Vector3 p_base = T_cam2base * p_cam;

    const auto &pos = latest_pose_.pose.position;
    tf2::Vector3 trans(pos.x, pos.y, pos.z);

    const auto &ori = latest_pose_.pose.orientation;
    tf2::Quaternion q;
    tf2::fromMsg(ori, q);

    tf2::Vector3 rotated = tf2::quatRotate(q, p_base);
    tf2::Vector3 global = trans + rotated;

    return {static_cast<float>(global.x()), static_cast<float>(global.y()),
            static_cast<float>(global.z())};
  }

  // New function to publish probes as RViz markers
  void publishProbeMarkers() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = latest_pose_.header.frame_id; // Use global frame (e.g., "map")
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "probes";
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

    // Add each probe as a point with color based on confidence
    for (const auto &p : tracked_probes_) {
        auto [x, y, z] = p.getAveragePosition();
        float confidence = p.getAverageConfidence();

        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = z;
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

  void probeCallback(const ProbeLocations::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received new probe locations");

    bool new_probe_found = false;
    for (size_t i = 0; i < msg->num_probes; ++i) {
      size_t base = i * 3;
      float lx = msg->probes[base + 0];
      float ly = msg->probes[base + 1];
      float lz = msg->probes[base + 2];
      float confidence = msg->classification_confidence[i];

      auto [gx, gy, gz] = transformToGlobal(lx, ly, lz);

      Probe incoming_probe(gx, gy, gz, confidence);
      bool matched_existing = false;
      for (Probe &tracked : tracked_probes_) {
        RCLCPP_INFO(get_logger(), "Incoming probe: %f %f %f", gx, gy, gz);
        auto [tx, ty, tz] = tracked.getAveragePosition();
        RCLCPP_INFO(get_logger(), "Tracked probe: %f %f %f", tx, ty, tz);
        RCLCPP_INFO(get_logger(), "Distance to existing probe: %f",
                    tracked.distanceTo(gx, gy, gz));

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

    // Existing ProbeData publishing logic (unchanged)
    if (new_probe_found) {
      ProbeData msg_out;
      msg_out.stamp = this->get_clock()->now();
      msg_out.probe_count = tracked_probes_.size();
      for (auto &p : tracked_probes_) {
        auto [ax, ay, az] = p.getAveragePosition();
        msg_out.x.push_back(ax);
        msg_out.y.push_back(ay);
        msg_out.z.push_back(az);
        msg_out.confidence.push_back(p.getAverageConfidence());
        msg_out.contribution.push_back(p.getCount());
      }
      publisher_->publish(msg_out);

      // Add marker publishing for RViz
      publishProbeMarkers();
    }
  }

  float merge_threshold_ = 0.3; // distance in meters to merge probes
  std::vector<Probe> tracked_probes_;
  rclcpp::Subscription<ProbeLocations>::SharedPtr subscription_;
  rclcpp::Publisher<ProbeData>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_; // New marker publisher
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  geometry_msgs::msg::PoseStamped latest_pose_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProbeFilteringNode>());
  rclcpp::shutdown();
  return 0;
}