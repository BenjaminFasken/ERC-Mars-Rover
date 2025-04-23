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
find_package(interfaces REQUIRED)

# 
find_package(tf2 REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED) 
find_package(tf2_ros REQUIRED) 

# Detect architecture
message(STATUS "Detected processor: ${CMAKE_SYSTEM_PROCESSOR}")

# Find CUDA with architecture-specific handling
find_package(CUDA 12.6 QUIET)
if(CUDA_FOUND)
  message(STATUS "CUDA found: ${CUDA_VERSION}")
else()
  # Fallback to manual paths based on architecture
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(CUDA_INCLUDE_DIRS /usr/local/cuda-12植物/include)
    set(CUDA_LIBRARIES /usr/local/cuda-12.6/lib64/libcudart.so)
  else()
    set(CUDA_INCLUDE_DIRS /usr/local/cuda-12.6/include)
    set(CUDA_LIBRARIES /usr/local/cuda-12.6/lib/libcudart.so)
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
  set(TENSORRT_DEFAULT_INCLUDE_DIR /usr/include/x86_64-linux-gnu)  # Updated
  set(TENSORRT_DEFAULT_LIB_DIR /usr/lib/x86_64-linux-gnu)         # Updated
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

# Add the probe_filter node
add_executable(probe_filtering_node src/probe_filtering.cpp)
ament_target_dependencies(probe_filtering_node rclcpp interfaces geometry_msgs tf2_geometry_msgs tf2 tf2_ros)


# Add the probe_localizer executable
# add_executable(probe_localizer src/probe_localizer.cpp)
# ament_target_dependencies(probe_localizer rclcpp sensor_msgs cv_bridge geometry_msgs)
# target_include_directories(probe_localizer PUBLIC ${OpenCV_INCLUDE_DIRS})
# target_link_libraries(probe_localizer ${OpenCV_LIBRARIES})

# Add the yolo executable with TensorRT and CUDA linking
# add_executable(yolo src/yolo.cpp)
# ament_target_dependencies(yolo rclcpp sensor_msgs cv_bridge geometry_msgs)
# target_include_directories(yolo PUBLIC
#   ${TENSORRT_INCLUDE_DIR}
#   ${CUDA_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
# )
# target_link_libraries(yolo
#   ${TENSORRT_LIBRARY}
#   ${TENSORRT_PLUGIN_LIBRARY}
#   ${CUDA_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

# Install the executables
# install(TARGETS probe_localizer
#   DESTINATION lib/${PROJECT_NAME}
# )
# install(TARGETS yolo
#   DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS
  probe_filtering_node
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

# Install model files
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install launch files (uncomment if needed)
#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Mark the package as complete
ament_package()