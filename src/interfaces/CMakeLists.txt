cmake_minimum_required(VERSION 3.8)
project(interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# Optional check for builtin_interfaces
if(NOT builtin_interfaces_FOUND)
  message(FATAL_ERROR "Could not find builtin_interfaces package")
endif()

# Generate message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ProbeSegmentation.msg"
  "msg/ProbeLocations.msg"
  "msg/ProbeData.msg"
  "srv/SetLedPattern.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs
)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()
