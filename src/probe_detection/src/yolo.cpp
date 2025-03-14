#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <fstream>
#include <memory>
#include <vector>

// TensorRT includes
#include <NvInfer.h>
#include <NvOnnxParser.h>

using namespace nvinfer1;

// Logger for TensorRT
class Logger : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char* msg) noexcept override {
    if (severity <= Severity::kWARNING) {  // Log warnings and errors
      RCLCPP_INFO(rclcpp::get_logger("YoloTensorRT"), "[TensorRT] %s", msg);
    }
  }
} gLogger;

class YoloTensorRTNode : public rclcpp::Node {
public:
  YoloTensorRTNode() : Node("yolo_tensorrt_node") {
    // Declare parameters
    this->declare_parameter<std::string>("engine_path", "/home/daroe/ERC-Mars-Rover/runs/detect/train9/weights/best.engine");

    // Get the engine file path
    std::string engine_path;
    this->get_parameter("engine_path", engine_path);

    // Load the TensorRT engine
    if (!loadEngine(engine_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load TensorRT engine.");
      throw std::runtime_error("Engine loading failed.");
    }

    RCLCPP_INFO(this->get_logger(), "TensorRT engine loaded successfully.");
  }

private:
  bool loadEngine(const std::string& engine_path) {
    // Read the engine file
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file) {
      RCLCPP_ERROR(this->get_logger(), "Could not open engine file: %s", engine_path.c_str());
      return false;
    }

    // Get file size
    engine_file.seekg(0, std::ios::end);
    size_t size = engine_file.tellg();
    engine_file.seekg(0, std::ios::beg);

    // Read the engine data
    std::vector<char> engine_data(size);
    engine_file.read(engine_data.data(), size);
    engine_file.close();

    // Create TensorRT runtime
    runtime_ = std::unique_ptr<nvinfer1::IRuntime>(createInferRuntime(gLogger));
    if (!runtime_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create TensorRT runtime.");
      return false;
    }

    // Deserialize the engine
    engine_ = std::unique_ptr<nvinfer1::ICudaEngine>(
        runtime_->deserializeCudaEngine(engine_data.data(), size));
    if (!engine_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to deserialize TensorRT engine.");
      return false;
    }

    // Create execution context
    context_ = std::unique_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
    if (!context_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create TensorRT execution context.");
      return false;
    }

    return true;
  }

  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<YoloTensorRTNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception occurred: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}