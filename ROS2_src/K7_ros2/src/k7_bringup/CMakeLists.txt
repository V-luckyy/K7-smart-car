cmake_minimum_required(VERSION 3.8)
project(k7_bringup)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# wjwwood serial 库（MIT）已 vendor 到 third_party/serial，不再依赖系统 serial 包
add_library(k7_serial_lib STATIC
  third_party/serial/src/serial.cc
  third_party/serial/src/impl/unix.cc
  third_party/serial/src/impl/list_ports/list_ports_linux.cc
)
target_include_directories(k7_serial_lib PUBLIC third_party/serial/include)

# ROS-only 11/24-byte bridge.
add_executable(serial_bridge src/serial_bridge.cpp)
ament_target_dependencies(serial_bridge
  rclcpp rcl_interfaces std_msgs sensor_msgs nav_msgs geometry_msgs
  tf2_ros
)
target_link_libraries(serial_bridge k7_serial_lib)

install(TARGETS serial_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch udev
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
