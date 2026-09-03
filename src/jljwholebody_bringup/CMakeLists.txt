cmake_minimum_required(VERSION 3.8)
project(jljwholebody_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/build_cannelloni.sh
    scripts/start_can_bridge.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
