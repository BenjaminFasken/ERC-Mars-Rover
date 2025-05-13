cmake_minimum_required(VERSION 3.10)  
project(navigation)
find_package(ament_cmake REQUIRED)

find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


# Add the probe_filter node
add_executable(probe_filtering_node src/probe_filtering.cpp)
ament_target_dependencies(probe_filtering_node rclcpp interfaces geometry_msgs tf2_geometry_msgs tf2 tf2_ros visualization_msgs)

install(TARGETS
  probe_filtering_node
  DESTINATION lib/${PROJECT_NAME}
)

# install scripts
install(PROGRAMS
  scripts/cmd_repeater.py
  scripts/path_visulizer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
ament_package()