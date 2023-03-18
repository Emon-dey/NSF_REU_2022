cmake_minimum_required(VERSION 3.5)
project(physics_module)

add_compile_options(-std=c++11)

find_package(colcon REQUIRED
  gazebo_ros
  roscpp
)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)

include(FindPkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
string(REPLACE ";" " " GAZEBO_CFLAGS "${GAZEBO_CFLAGS}")
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CFLAGS}")

