#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/probe_locations.hpp"
#include "interfaces/msg/probe_data.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>                   // for stamped pose messages
#include <tf2/LinearMath/Quaternion.h>                          // for tf2::Quaternion 
#include <tf2/LinearMath/Vector3.h>                             // for tf2::Vector3
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>              // for conversions between geometry_msgs and tf2 types 
#include <cmath>                                               // for std::sqrt

using ProbeLocations = interfaces::msg::ProbeLocations;
using ProbeData      = interfaces::msg::ProbeData;

// General class to hold probe information
class Probe
{
public:
  Probe(float x, float y, float z, float confidence) // initialization with coordinates and confidence
  : x_sum_(x), y_sum_(y), z_sum_(z), confidence_sum_(confidence), count_(1) {} // count is one for the first probe

  // Update the probe with new data
  // Will be called when a new probe is detected
  void update(float x, float y, float z, float confidence)
  {
    x_sum_ += x;
    y_sum_ += y;
    z_sum_ += z;
    confidence_sum_ += confidence;
    count_++;
  }

  // Calculate the distance to another point
  // Will be used to check if the new probe is close enough to an existing one
  float distanceTo(float x, float y, float z) const
  {
    auto [avg_x, avg_y, avg_z] = getAveragePosition();
    float dx = avg_x - x;
    float dy = avg_y - y;
    float dz = avg_z - z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  // Get the average position
  // Serves as the 'actual' position of the probe
  std::tuple<float, float, float> getAveragePosition() const
  {
    return {
      x_sum_ / count_,
      y_sum_ / count_,
      z_sum_ / count_
    };
  }

  // Get the average confidence
  // This is the average confidence of the probe over all detections //! might need tweaking later
  float getAverageConfidence() const
  {
    return confidence_sum_ / count_;
  }

  // Get the count of detections
  int getCount() const { return count_; }

// all variables saved in class
private:
  float x_sum_;
  float y_sum_;
  float z_sum_;
  float confidence_sum_;
  int count_;
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
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "localization_pose", 10,
      std::bind(&ProbeFilteringNode::poseCallback, this, std::placeholders::_1)); 

    publisher_ = this->create_publisher<ProbeData>("filtered_probes", 10);

    RCLCPP_INFO(this->get_logger(), "Probe filtering Node initialized.");
  }

private:
  bool pose_received_ = false;

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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
    const auto &pos = latest_pose_.pose.position;
    tf2::Vector3 trans(pos.x, pos.y, pos.z);

    // orientation quaternion from pose
    const auto &ori = latest_pose_.pose.orientation;
    tf2::Quaternion q;
    tf2::fromMsg(ori, q);                                       // convert geometry_msgs→tf2 

    // rotate local vector
    tf2::Vector3 local(lx, ly, lz);
    tf2::Vector3 rotated = tf2::quatRotate(q, local);            // rotate via quaternion 

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

      //! change back once the global pose is actually being broadcasted ----------------------------------------------------
      // convert to global frame before merging
      // auto [gx, gy, gz] = transformToGlobal(lx, ly, lz);

      auto gx = lx; // for now, no transformation
      auto gy = ly; // for now, no transformation
      auto gz = lz; // for now, no transformation

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
  
  // pose subscription
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  geometry_msgs::msg::PoseStamped latest_pose_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProbeFilteringNode>());
  rclcpp::shutdown();
  return 0;
}
