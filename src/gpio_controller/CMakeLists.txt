cmake_minimum_required(VERSION 3.8)
project(gpio_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(interfaces REQUIRED)

# Install Python scripts as executables
install(
  PROGRAMS
    scripts/gpio_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
