cmake_minimum_required(VERSION 3.8)
project(lidar_nodes)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Building with the system compiler (not a conda-provided one) against
# the pixi/robostack-installed ROS2 libs - neither LD_LIBRARY_PATH nor
# CMAKE_PREFIX_PATH are exported by `pixi run`, so link_directories +
# a baked-in rpath are needed for both the link step (e.g. liblttng-ust,
# an rclcpp dependency) and running the built binaries directly (this
# package isn't sourced via install/setup.bash).
link_directories($ENV{CONDA_PREFIX}/lib)
set(CMAKE_INSTALL_RPATH $ENV{CONDA_PREFIX}/lib)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# The repo's ESP32 firmware source tree, for dfrobot_matrix_lidar.inc -
# reused unmodified rather than re-derived in C++ a second time (it
# already is C++; only the Arduino/Wire APIs it calls needed shimming,
# see include/arduino_compat/).
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)

set(ARDUINO_COMPAT_SOURCES src/wire_linux_i2c.cpp)
set(ARDUINO_COMPAT_INCLUDES
  include
  include/arduino_compat
)

# DFRobot 8x8 Matrix ToF (VL53L7CX) sensor node.
add_executable(lidar_matrix_node
  src/lidar_matrix_node.cpp
  ${ARDUINO_COMPAT_SOURCES}
)
target_include_directories(lidar_matrix_node PRIVATE
  ${ARDUINO_COMPAT_INCLUDES}
  ${REPO_ROOT}/src/apps
)
ament_target_dependencies(lidar_matrix_node rclcpp sensor_msgs)

# VL53L0X single-point ToF sensor node.
add_executable(lidar_point_node
  src/lidar_point_node.cpp
  third_party/pololu_vl53l0x/VL53L0X.cpp
  ${ARDUINO_COMPAT_SOURCES}
)
target_include_directories(lidar_point_node PRIVATE
  ${ARDUINO_COMPAT_INCLUDES}
  third_party/pololu_vl53l0x
)
ament_target_dependencies(lidar_point_node rclcpp sensor_msgs)

install(TARGETS lidar_matrix_node lidar_point_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
