cmake_minimum_required(VERSION 3.5)
project(interfaces)
    
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
if(NOT builtin_interfaces_FOUND)
message(FATAL_ERROR "Could not find builtin_interfaces package")
endif()

# Generate message files
rosidl_generate_interfaces(${PROJECT_NAME}
"msg/ProbeSegmentation.msg"
DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs)


ament_export_dependencies(rosidl_default_runtime)

# find dependencies
find_package(ament_cmake REQUIRED)


ament_package()