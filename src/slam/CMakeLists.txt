cmake_minimum_required(VERSION 3.10)  
project(slam)
find_package(ament_cmake REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/path_node.py
  scripts/map_filter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
ament_package()