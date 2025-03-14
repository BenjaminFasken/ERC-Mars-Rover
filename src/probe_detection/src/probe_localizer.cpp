#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <mutex>
#include <vector>
#include <stdexcept>

// Define Detection struct
struct Detection {
    float xmin, ymin, xmax, ymax;
};

class ProbeLocalizer : public rclcpp::Node {
public:
    ProbeLocalizer() : Node("probe_localizer") {
        // Initialize ROS 2 subscribers and publisher
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/zed/zed_node/rgb/image_rect_color", 10,
            std::bind(&ProbeLocalizer::imageCallback, this, std::placeholders::_1));
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/zed/zed_node/depth/depth_registered", 10,
            std::bind(&ProbeLocalizer::depthCallback, this, std::placeholders::_1));
        location_pub_ = create_publisher<geometry_msgs::msg::Point>("probe_locations", 10);

        // Load the ONNX model
        std::string model_path = "/home/daroe/ERC-Mars-Rover/runs/detect/train7/weights/best.onnx";
        try {
            net_ = cv::dnn::readNetFromONNX(model_path);
            if (net_.empty()) {
                throw std::runtime_error("Failed to load ONNX model: network is empty");
            }
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU); // Use DNN_TARGET_CUDA for GPU
            RCLCPP_INFO(this->get_logger(), "Loaded YOLOv12 model from %s", model_path.c_str());
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load ONNX model: %s", e.what());
            throw std::runtime_error("Model loading failed");
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr location_pub_;
    cv::dnn::Net net_;
    cv::Mat latest_depth_map_;
    std::mutex depth_mutex_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;

            std::vector<Detection> detections = runYoloInference(image);
            std::vector<geometry_msgs::msg::Point> locations = computeProbeLocations(detections);

            for (const auto& loc : locations) {
                location_pub_->publish(loc);
                RCLCPP_INFO(this->get_logger(), "Probe at: x=%.2f, y=%.2f, z=%.2f", loc.x, loc.y, loc.z);
            }
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
        }
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            std::lock_guard<std::mutex> lock(depth_mutex_);
            latest_depth_map_ = cv_ptr->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Depth cv_bridge error: %s", e.what());
        }
    }

    std::vector<Detection> runYoloInference(const cv::Mat& image) {
        std::vector<Detection> detections;

        // Preprocess image
        cv::Mat blob = cv::dnn::blobFromImage(image, 1.0 / 255.0, cv::Size(416, 416), cv::Scalar(0, 0, 0), true, false);
        net_.setInput(blob);

        // Run inference
        std::vector<cv::Mat> outputs;
        std::vector<std::string> output_names = net_.getUnconnectedOutLayersNames();
        net_.forward(outputs, output_names);

        // Post-process (assuming YOLOv12 format similar to YOLOv8: [x_center, y_center, width, height, confidence, class_scores...])
        float conf_threshold = 0.25;
        int img_width = image.cols;
        int img_height = image.rows;

        for (const auto& output : outputs) {
            for (int i = 0; i < output.rows; i++) {
                float confidence = output.at<float>(i, 4);
                if (confidence < conf_threshold) continue;

                float x_center = output.at<float>(i, 0) * img_width;
                float y_center = output.at<float>(i, 1) * img_height;
                float width = output.at<float>(i, 2) * img_width;
                float height = output.at<float>(i, 3) * img_height;

                Detection det;
                det.xmin = x_center - width / 2.0;
                det.ymin = y_center - height / 2.0;
                det.xmax = x_center + width / 2.0;
                det.ymax = y_center + height / 2.0;

                detections.push_back(det);
            }
        }

        return detections;
    }

    std::vector<geometry_msgs::msg::Point> computeProbeLocations(const std::vector<Detection>& detections) {
        std::vector<geometry_msgs::msg::Point> locations;
        std::lock_guard<std::mutex> lock(depth_mutex_);

        if (latest_depth_map_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No depth map available yet.");
            return locations;
        }

        for (const auto& det : detections) {
            float center_x = (det.xmin + det.xmax) / 2.0;
            float center_y = (det.ymin + det.ymax) / 2.0;

            float depth = latest_depth_map_.at<float>(static_cast<int>(center_y), static_cast<int>(center_x));
            if (std::isnan(depth) || depth <= 0) continue;

            float fx = 700.0; // Replace with ZED intrinsics
            float fy = 700.0;
            float cx = latest_depth_map_.cols / 2.0;
            float cy = latest_depth_map_.rows / 2.0;

            geometry_msgs::msg::Point loc;
            loc.z = depth;
            loc.x = (center_x - cx) * depth / fx;
            loc.y = (center_y - cy) * depth / fy;

            locations.push_back(loc);
        }

        return locations;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<ProbeLocalizer>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to start ProbeLocalizer: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}