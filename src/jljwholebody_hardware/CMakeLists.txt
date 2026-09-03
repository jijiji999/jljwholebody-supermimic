cmake_minimum_required(VERSION 3.8)
project(jljwholebody_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/dm_motor_driver.cpp
  src/dm_motor_manager.cpp
  src/dm_motor_protocol.cpp
  src/jljwholebody_system.cpp
  src/parallel_kinematics.cpp
  src/socketcan_transport.cpp
  src/waist_parallel_solver.cpp
  src/waist_parallel_yaml.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  yaml-cpp
)

pluginlib_export_plugin_description_file(hardware_interface jljwholebody_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES jljwholebody_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  ament_index_cpp
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  Eigen3
)

ament_package()
