#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <chrono>

const int IMAGE_SIZE = 640;
const float CONF_THRESHOLD = 0.5f;
const float IOU_THRESHOLD = 0.4f;

struct Detection {
    cv::Rect box;
    float conf;
    int class_id;
};

cv::Mat letterbox(const cv::Mat& img, int size) {
    if (img.empty()) {
        throw std::runtime_error("Input image is empty in letterbox");
    }
    int width = img.cols;
    int height = img.rows;
    float r = std::min(static_cast<float>(size) / width, static_cast<float>(size) / height);
    int new_w = static_cast<int>(std::round(width * r));
    int new_h = static_cast<int>(std::round(height * r));
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    cv::Mat letterboxed(size, size, CV_8UC3, cv::Scalar(114, 114, 114));
    int top = (size - new_h) / 2;
    int left = (size - new_w) / 2;
    cv::Rect roi(left, top, new_w, new_h);
    resized.copyTo(letterboxed(roi));
    return letterboxed;
}

template <typename T>
T clamp(T value, T low, T high) {
    return std::max(low, std::min(value, high));
}

class ZEDImageVisualizer : public rclcpp::Node {
public:
    ZEDImageVisualizer() : Node("zed_image_visualizer"), visualizing_(false), running_(true) {
        subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "/zed/zed_node/rgb/image_rect_color", 10,
            std::bind(&ZEDImageVisualizer::image_callback, this, std::placeholders::_1));

        try {
            net_ = cv::dnn::readNetFromONNX("/home/daroe/ERC-Mars-Rover/src/probe_detection/models/many_probes_weights.onnx");
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            net_.enableFusion(false);
            RCLCPP_INFO(this->get_logger(), "Model loaded successfully");
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load model: %s", e.what());
            rclcpp::shutdown();
        }

        key_thread_ = std::thread(&ZEDImageVisualizer::key_listener, this);
        RCLCPP_INFO(this->get_logger(), "Press SPACE to start/pause visualization, ESC to exit.");
    }

    ~ZEDImageVisualizer() {
        running_ = false;
        if (key_thread_.joinable()) key_thread_.join();
        cv::destroyAllWindows();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!visualizing_) return;

        RCLCPP_INFO(this->get_logger(), "Processing new image message");

        try {
            // Validate and log message details
            if (!msg) {
                RCLCPP_ERROR(this->get_logger(), "Received null image message");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Message details: width=%u, height=%u, encoding=%s, data_size=%zu",
                        msg->width, msg->height, msg->encoding.c_str(), msg->data.size());
            if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
                RCLCPP_ERROR(this->get_logger(), "Invalid image message: empty data or zero dimensions");
                return;
            }

            // Convert to OpenCV with encoding handling
            cv_bridge::CvImagePtr cv_ptr;
            std::string encoding = msg->encoding;
            if (encoding == "bgr8") {
                cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            } else if (encoding == "bgra8") {
                cv_ptr = cv_bridge::toCvCopy(msg, "bgra8");
                cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGRA2BGR);
            } else {
                RCLCPP_WARN(this->get_logger(), "Unsupported encoding '%s', attempting conversion to bgr8", encoding.c_str());
                cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            }

            cv::Mat cv_image = cv_ptr->image;
            if (cv_image.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Converted image is empty");
                return;
            }

            // Rotate
            cv::rotate(cv_image, cv_image, cv::ROTATE_180);
            RCLCPP_INFO(this->get_logger(), "Image size: %d x %d", cv_image.cols, cv_image.rows);

            // Preprocess
            cv::Mat input_image = letterbox(cv_image, IMAGE_SIZE);
            RCLCPP_INFO(this->get_logger(), "Input image size after letterbox: %d x %d", input_image.cols, input_image.rows);

            cv::Mat blob = cv::dnn::blobFromImage(input_image, 1.0 / 255.0, 
                                                 cv::Size(IMAGE_SIZE, IMAGE_SIZE), 
                                                 cv::Scalar(0, 0, 0), true, false);
            RCLCPP_INFO(this->get_logger(), "Blob created with dims: %d", blob.dims);

            // Inference
            net_.setInput(blob);
            std::vector<cv::Mat> outs;
            auto start = std::chrono::high_resolution_clock::now();
            net_.forward(outs, net_.getUnconnectedOutLayersNames());
            auto end = std::chrono::high_resolution_clock::now();
            RCLCPP_INFO(this->get_logger(), "Inference time: %ld ms", 
                        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

            for (size_t i = 0; i < outs.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "Output %zu shape: [%d, %d, %d]", 
                            i, outs[i].size[0], outs[i].size[1], outs[i].size[2]);
            }

            //print outs
            for (size_t i = 0; i < outs.size(); ++i) {
                const cv::Mat& detections = outs[i];
                if (detections.dims != 3 || detections.size[0] != 1 || detections.size[1] != 5) {
                    RCLCPP_ERROR(this->get_logger(), "Unexpected shape: [%d, %d, %d]",
                                detections.size[0], detections.size[1], detections.size[2]);
                    continue;
                }

                int num_detections = detections.size[2];
                for (int i = 0; i < std::min(5, num_detections); ++i) {
                    RCLCPP_INFO(this->get_logger(), "Layer %zu detection %d: x=%f, y=%f, w=%f, h=%f, conf=%f",
                                i, detections.at<float>(0, 0, i), detections.at<float>(0, 1, i),
                                detections.at<float>(0, 2, i), detections.at<float>(0, 3, i),
                                detections.at<float>(0, 4, i));
                }
            }

            // Postprocess


            // Process detections
            //std::vector<Detection> detections = process_detections(outs, cv_image, input_image);

            // Visualize
            // for (const auto& det : detections) {
            //     cv::rectangle(cv_image, det.box, cv::Scalar(0, 255, 0), 2);
            //     std::string label = "Probe: " + std::to_string(det.conf).substr(0, 4);
            //     cv::putText(cv_image, label, cv::Point(det.box.x, det.box.y - 5),
            //                 cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            // }

            // cv::imshow("YOLOv11 Detections", cv_image);
            // cv::waitKey(1);
        } 
        catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } 
        catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Standard exception: %s", e.what());
        }
    }

    void key_listener() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        while (running_) {
            char key = getchar();
            if (key == ' ') {
                visualizing_ = !visualizing_;
                RCLCPP_INFO(this->get_logger(), visualizing_ ? "Visualizing..." : "Paused...");
            } 
            else if (key == 27) {
                RCLCPP_INFO(this->get_logger(), "Exiting...");
                running_ = false;
                rclcpp::shutdown();
                break;
            }
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::dnn::Net net_;
    bool visualizing_;
    bool running_;
    std::thread key_thread_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZEDImageVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}