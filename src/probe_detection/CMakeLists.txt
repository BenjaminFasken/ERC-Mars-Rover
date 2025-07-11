cmake_minimum_required(VERSION 3.5)
project(probe_detection)

# Default to C++17 for TensorRT compatibility
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies (ensure ROS environment is sourced)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
set(cv_bridge_DIR /opt/ros/humble/lib/cmake/cv_bridge)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Detect architecture
message(STATUS "Detected processor: ${CMAKE_SYSTEM_PROCESSOR}")

# Use CUDA_VERSION from build argument, default to 12.6
if(NOT CUDA_VERSION)
  set(CUDA_VERSION 12.6)
endif()
message(STATUS "Using CUDA version: ${CUDA_VERSION}")

# Find CUDA with architecture-specific handling
find_package(CUDA ${CUDA_VERSION} QUIET)
if(CUDA_FOUND)
  message(STATUS "CUDA found: ${CUDA_VERSION}")
else()
  # Fallback to manual paths based on architecture
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(CUDA_INCLUDE_DIRS /usr/local/cuda-${CUDA_VERSION}/include)
    set(CUDA_LIBRARIES /usr/local/cuda-${CUDA_VERSION}/lib64/libcudart.so)
  else()
    set(CUDA_INCLUDE_DIRS /usr/local/cuda-${CUDA_VERSION}/include)
    set(CUDA_LIBRARIES /usr/local/cuda-${CUDA_VERSION}/lib64/libcudart.so)
  endif()
  # Verify manual paths
  if(NOT EXISTS ${CUDA_INCLUDE_DIRS}/cuda.h)
    message(FATAL_ERROR "CUDA headers not found at ${CUDA_INCLUDE_DIRS}. Adjust CUDA_INCLUDE_DIRS.")
  endif()
  if(NOT EXISTS ${CUDA_LIBRARIES})
    message(FATAL_ERROR "CUDA library not found at ${CUDA_LIBRARIES}. Adjust CUDA_LIBRARIES.")
  endif()
endif()

# Find TensorRT with architecture-specific paths
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  # ARM (e.g., Jetson) default paths
  set(TENSORRT_DEFAULT_INCLUDE_DIR /usr/include/aarch64-linux-gnu)
  set(TENSORRT_DEFAULT_LIB_DIR /usr/lib/aarch64-linux-gnu)
else()
  # Non-ARM (e.g., x86_64) default paths
  set(TENSORRT_DEFAULT_INCLUDE_DIR /usr/include/x86_64-linux-gnu)
  set(TENSORRT_DEFAULT_LIB_DIR /usr/lib/x86_64-linux-gnu)
endif()

# Allow override via -D flag
if(NOT TENSORRT_INCLUDE_DIR)
  set(TENSORRT_INCLUDE_DIR ${TENSORRT_DEFAULT_INCLUDE_DIR})
endif()
if(NOT TENSORRT_LIB_DIR)
  set(TENSORRT_LIB_DIR ${TENSORRT_DEFAULT_LIB_DIR})
endif()

# Find TensorRT libraries
find_library(TENSORRT_LIBRARY nvinfer HINTS ${TENSORRT_LIB_DIR})
find_library(TENSORRT_PLUGIN_LIBRARY nvinfer_plugin HINTS ${TENSORRT_LIB_DIR})

if(NOT TENSORRT_LIBRARY OR NOT TENSORRT_PLUGIN_LIBRARY)
  message(FATAL_ERROR "TensorRT libraries not found. Searched in ${TENSORRT_LIB_DIR}. Adjust TENSORRT_LIB_DIR.")
endif()

if(NOT EXISTS ${TENSORRT_INCLUDE_DIR}/NvInfer.h)
  message(FATAL_ERROR "TensorRT headers not found at ${TENSORRT_INCLUDE_DIR}. Adjust TENSORRT_INCLUDE_DIR.")
endif()

message(STATUS "Using TensorRT include: ${TENSORRT_INCLUDE_DIR}")
message(STATUS "Using TensorRT libs: ${TENSORRT_LIBRARY}, ${TENSORRT_PLUGIN_LIBRARY}")

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/ImageCaptureNode.py
  scripts/detect_probe.py
  scripts/Capture_test_set.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install model files
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Package is complete
ament_package()