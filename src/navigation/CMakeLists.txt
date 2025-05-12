cmake_minimum_required(VERSION 3.10)  
project(navigation)
find_package(ament_cmake REQUIRED)

# install scripts
install(PROGRAMS
  scripts/cmd_repeater.py
  scripts/path_visulizer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
ament_package()