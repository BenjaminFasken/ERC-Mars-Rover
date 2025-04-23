cmake_minimum_required(VERSION 3.8)
project(gpio_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Add the service definition
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetLedPattern.srv"
  TARGET_NAME ${PROJECT_NAME}_interfaces # Keep target name
)

# Make the python installation depend on the interface generation
ament_python_install_package(
    DEPENDENCIES ${PROJECT_NAME}_interfaces
)

# Also add dependency to ament_package
ament_package(
    DEPENDENCIES ${PROJECT_NAME}_interfaces # Add this line
)