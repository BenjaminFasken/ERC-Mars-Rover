cmake_minimum_required(VERSION 3.5)
project(probe_detection)

# Default to C++14 (TensorRT may require C++17, so we'll set it explicitly)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)  # Updated to C++17 for TensorRT compatibility
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Find TensorRT
set(TENSORRT_LIB_DIR /usr/lib/x86_64-linux-gnu)  # Confirmed from your nm output
set(TENSORRT_INCLUDE_DIR /usr/include/x86_64-linux-gnu)  # Typical location for headers

find_library(TENSORRT_LIBRARY nvinfer HINTS ${TENSORRT_LIB_DIR})
find_library(TENSORRT_PLUGIN_LIBRARY nvinfer_plugin HINTS ${TENSORRT_LIB_DIR})

if(NOT TENSORRT_LIBRARY OR NOT TENSORRT_PLUGIN_LIBRARY)
  message(FATAL_ERROR "TensorRT libraries not found. Please install TensorRT or adjust TENSORRT_LIB_DIR.")
endif()

if(NOT EXISTS ${TENSORRT_INCLUDE_DIR}/NvInfer.h)
  message(FATAL_ERROR "TensorRT headers not found. Please adjust TENSORRT_INCLUDE_DIR.")
endif()

# Add the probe_localizer executable (unchanged)
add_executable(probe_localizer src/probe_localizer.cpp)
ament_target_dependencies(probe_localizer rclcpp sensor_msgs cv_bridge geometry_msgs OpenCV)

# Add the yolo executable with TensorRT linking
add_executable(yolo src/yolo.cpp)
ament_target_dependencies(yolo rclcpp sensor_msgs cv_bridge geometry_msgs OpenCV)
target_include_directories(yolo PUBLIC
  ${TENSORRT_INCLUDE_DIR}
)
target_link_libraries(yolo
  ${TENSORRT_LIBRARY}
  ${TENSORRT_PLUGIN_LIBRARY}
)

# Install the executables
install(TARGETS probe_localizer
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS yolo
  DESTINATION lib/${PROJECT_NAME}
)

# Include directories
include_directories(include)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/ImageCaptureNode.py
  scripts/detect_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (uncomment if needed)
#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Mark the package as complete
ament_package()