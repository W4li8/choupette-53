cmake_minimum_required(VERSION 3.8)
project(lidar_matrix_node)

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
# an rclcpp dependency) and running the built binary directly (this
# package isn't sourced via install/setup.bash).
link_directories($ENV{CONDA_PREFIX}/lib)
set(CMAKE_INSTALL_RPATH $ENV{CONDA_PREFIX}/lib)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# The repo's ESP32 firmware source tree, for dfrobot_matrix_lidar.inc -
# reused unmodified rather than re-derived in C++ a second time (it
# already is C++; only the Arduino/Wire APIs it calls needed shimming,
# see include/arduino_compat/).
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)

add_executable(lidar_matrix_node
  src/lidar_matrix_node.cpp
  src/wire_linux_i2c.cpp
)
target_include_directories(lidar_matrix_node PRIVATE
  include
  include/arduino_compat
  ${REPO_ROOT}/src/apps
)
ament_target_dependencies(lidar_matrix_node rclcpp sensor_msgs)

install(TARGETS lidar_matrix_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
