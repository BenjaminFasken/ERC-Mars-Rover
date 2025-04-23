#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/probe_locations.hpp"
#include "interfaces/msg/probe_data.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>    // for PoseWithCovarianceStamped :contentReference[oaicite:0]{index=0}
#include <tf2/LinearMath/Quaternion.h>                          // for tf2::Quaternion :contentReference[oaicite:1]{index=1}
#include <tf2/LinearMath/Vector3.h>                             // for tf2::Vector3
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>                // for conversions between geometry_msgs and tf2 types :contentReference[oaicite:2]{index=2}

using ProbeLocations = interfaces::msg::ProbeLocations;
using ProbeData      = interfaces::msg::ProbeData;

// (unchanged) Probe class …
class Probe
{
  // … your existing Probe implementation …
};

class ProbeFilteringNode : public rclcpp::Node
{
public:
  ProbeFilteringNode()
  : Node("probe_filtering_node")
  {
    // subscribe vision-system detections
    subscription_ = this->create_subscription<ProbeLocations>(
      "probe_detections", 10,
      std::bind(&ProbeFilteringNode::probeCallback, this, std::placeholders::_1));

    // subscribe global pose
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "localization_pose", 10,
      std::bind(&ProbeFilteringNode::poseCallback, this, std::placeholders::_1));  // :contentReference[oaicite:3]{index=3}

    publisher_ = this->create_publisher<ProbeData>("filtered_probes", 10);

    RCLCPP_INFO(this->get_logger(), "Probe filtering Node initialized.");
  }

private:
  // --- pose subscription & cache ---
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  geometry_msgs::msg::PoseWithCovarianceStamped latest_pose_;
  bool pose_received_ = false;

  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    latest_pose_ = *msg;
    pose_received_ = true;
    RCLCPP_DEBUG(get_logger(), "Received new localization_pose");  // for debug
  }

  // --- transform helper ---
  std::tuple<float, float, float> transformToGlobal(float lx, float ly, float lz)
  {
    if (!pose_received_) {
      RCLCPP_WARN(get_logger(), "No global pose yet; returning local coords unchanged");
      return {lx, ly, lz};
    }

    // translation from pose
    const auto &pos = latest_pose_.pose.pose.position;
    tf2::Vector3 trans(pos.x, pos.y, pos.z);

    // orientation quaternion from pose
    const auto &ori = latest_pose_.pose.pose.orientation;
    tf2::Quaternion q;
    tf2::fromMsg(ori, q);                                       // convert geometry_msgs→tf2 :contentReference[oaicite:4]{index=4}

    // rotate local vector
    tf2::Vector3 local(lx, ly, lz);
    tf2::Vector3 rotated = tf2::quatRotate(q, local);            // rotate via quaternion :contentReference[oaicite:5]{index=5}

    // translate into global
    tf2::Vector3 global = trans + rotated;

    return {
      static_cast<float>(global.x()),
      static_cast<float>(global.y()),
      static_cast<float>(global.z())
    };
  }

  // --- vision callback, now using global coords ---
  void probeCallback(const ProbeLocations::SharedPtr msg)
  {
    bool new_probe_found = false;
    for (size_t i = 0; i < msg->num_probes; ++i) {
      size_t base = i * 3;
      float lx = msg->probes[base + 0];
      float ly = msg->probes[base + 1];
      float lz = msg->probes[base + 2];
      float confidence = msg->classification_confidence[i];

      // convert to global frame before merging
      auto [gx, gy, gz] = transformToGlobal(lx, ly, lz);

      Probe incoming_probe(gx, gy, gz, confidence);
      bool matched_existing = false;
      for (Probe &tracked : tracked_probes_) {
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
    }
  }

  int merge_threshold_ = 10;
  std::vector<Probe> tracked_probes_;
  rclcpp::Subscription<ProbeLocations>::SharedPtr subscription_;
  rclcpp::Publisher<ProbeData>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProbeFilteringNode>());
  rclcpp::shutdown();
  return 0;
}
