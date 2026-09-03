cmake_minimum_required(VERSION 3.8)
project(jljwholebody_deploy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jljwholebody_hardware REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(_onnxruntime_arch "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(_onnxruntime_arch "x64")
else()
  message(FATAL_ERROR "Unsupported ONNX Runtime architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(
  ONNXRUNTIME_ROOT
  "$ENV{ONNXRUNTIME_ROOT}"
  CACHE PATH "ONNX Runtime distribution root"
)
if(NOT ONNXRUNTIME_ROOT)
  get_filename_component(
    ONNXRUNTIME_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../unitree_rl_mjlab/deploy/thirdparty/onnxruntime-linux-${_onnxruntime_arch}-1.22.0"
    ABSOLUTE
  )
endif()
find_path(
  ONNXRUNTIME_INCLUDE_DIR
  onnxruntime_cxx_api.h
  PATHS "${ONNXRUNTIME_ROOT}/include"
  NO_DEFAULT_PATH
  REQUIRED
)
find_file(
  ONNXRUNTIME_LIBRARY
  NAMES libonnxruntime.so.1 libonnxruntime.so.1.22.0
  PATHS "${ONNXRUNTIME_ROOT}/lib"
  NO_DEFAULT_PATH
  REQUIRED
)
get_filename_component(ONNXRUNTIME_LIBRARY_REAL "${ONNXRUNTIME_LIBRARY}" REALPATH)

add_library(wholebody_policy_runtime STATIC
  src/wholebody_policy.cpp
)
target_compile_features(wholebody_policy_runtime PUBLIC cxx_std_17)
target_include_directories(wholebody_policy_runtime
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${ONNXRUNTIME_INCLUDE_DIR}
)
target_link_libraries(wholebody_policy_runtime
  PUBLIC
    ${ONNXRUNTIME_LIBRARY}
)

add_executable(jljwholebody_deploy_node
  src/jljwholebody_deploy_node.cpp
)
target_compile_features(jljwholebody_deploy_node PUBLIC cxx_std_17)
target_include_directories(jljwholebody_deploy_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${jljwholebody_hardware_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
)
target_link_libraries(jljwholebody_deploy_node
  Eigen3::Eigen
  ${jljwholebody_hardware_LIBRARIES}
  wholebody_policy_runtime
  Threads::Threads
  yaml-cpp
)
set_target_properties(jljwholebody_deploy_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/.."
)
ament_target_dependencies(jljwholebody_deploy_node
  ament_index_cpp
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
)

install(
  TARGETS
    wholebody_policy_runtime
    jljwholebody_deploy_node
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Keep the installed workspace runnable without the training repository or a Python env.
install(
  FILES ${ONNXRUNTIME_LIBRARY_REAL}
  DESTINATION lib
  RENAME libonnxruntime.so.1
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  FILES launch/deploy.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
