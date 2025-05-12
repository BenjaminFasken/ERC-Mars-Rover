cmake_minimum_required(VERSION 3.10)  
project(slam)
find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/path_node.py
    scripts/map_filter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
ament_package()