#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/probe_locations.hpp"  // Adjust these paths as needed
#include "interfaces/msg/probe_data.hpp"         // Our custom ProbeData message

#include <cmath>
#include <tuple>
#include <vector>

using std::placeholders::_1;
using ProbeLocations = interfaces::msg::ProbeLocations;
using ProbeData = interfaces::msg::ProbeData;  

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

  float getAverageConfidence() const
  {
    return confidence_sum_ / count_;
  }

  int getCount() const { return count_; }

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
    subscription_ = this->create_subscription<ProbeLocations>(
      "probe_detections", 10,
      std::bind(&ProbeFilteringNode::probeCallback, this, _1));

    publisher_ = this->create_publisher<ProbeData>("filtered_probes", 10);

    RCLCPP_INFO(this->get_logger(), "Probe filtering Node initialized.");
  }

private:
  void probeCallback(const ProbeLocations::SharedPtr msg)
  {
    bool new_probe_found = false;  
    for (size_t i = 0; i < msg->num_probes; ++i) {
      size_t base = i * 3;
      float x = msg->probes[base + 0];
      float y = msg->probes[base + 1];
      float z = msg->probes[base + 2];
      float confidence = msg->classification_confidence[i];  
      Probe incoming_probe(x, y, z, confidence);  
      bool matched_existing = false;

      for (Probe& tracked : tracked_probes_) {
        if (tracked.distanceTo(x, y, z) < merge_threshold_) {
          tracked.update(x, y, z, confidence);  // Merge new data into tracked probe
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
      // Build the ProbeData message using lists to hold all found probe information at once.
      ProbeData msg_out;
      msg_out.stamp = this->get_clock()->now();

      // For every tracked probe, compute the average position, confidence, and detection count.
      for (const Probe& p : tracked_probes_) {
        auto [avg_x, avg_y, avg_z] = p.getAveragePosition();
        msg_out.x.push_back(avg_x);
        msg_out.y.push_back(avg_y);
        msg_out.z.push_back(avg_z);
        msg_out.confidence.push_back(p.getAverageConfidence());
        msg_out.count.push_back(p.getCount());
      }

      publisher_->publish(msg_out);
    }
  }

  int merge_threshold_ = 10; // Example threshold, adjust as needed
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
