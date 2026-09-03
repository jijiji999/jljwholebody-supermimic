cmake_minimum_required(VERSION 3.8)
project(dm_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(serial REQUIRED)

include_directories(
  include
)

add_executable(dm_imu_node
  src/bsp_crc.cpp
  src/imu_stream_parser.cpp
  src/imu_driver.cpp
  src/test_imu.cpp
)
target_compile_features(dm_imu_node PUBLIC cxx_std_17)

ament_target_dependencies(dm_imu_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  serial
)

install(TARGETS
  dm_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
