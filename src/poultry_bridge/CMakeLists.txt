cmake_minimum_required(VERSION 3.8)
project(poultry_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the poultry_bridge Python module (makes it importable)
ament_python_install_package(${PROJECT_NAME})

# Install thin wrapper scripts as ROS 2 executables
install(PROGRAMS
  scripts/uds_bridge_node
  scripts/uds_client
  scripts/uds_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install proto sources for reference
install(DIRECTORY proto/
  DESTINATION share/${PROJECT_NAME}/proto
)

ament_package()
