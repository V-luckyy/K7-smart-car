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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(k7_msgs REQUIRED)

# wjwwood serial 库（MIT）已 vendor 到 third_party/serial，不再依赖系统 serial 包
add_library(k7_serial_lib STATIC
  third_party/serial/src/serial.cc
  third_party/serial/src/impl/unix.cc
  third_party/serial/src/impl/list_ports/list_ports_linux.cc
)
target_include_directories(k7_serial_lib PUBLIC third_party/serial/include)

# k7_serial_node: 移植自 wheeltec_ros2 turn_on_wheeltec_robot 的 wheeltec_robot_node
# 改动点：设备名 /dev/k7_controller、新增 cmd_vel 看门狗(cmd_vel_timeout_ms 默认 500，超时发零速)、删 v650 死代码
add_executable(k7_serial_node src/k7_serial_node.cpp src/Quaternion_Solution.cpp)
target_include_directories(k7_serial_node PRIVATE include)
ament_target_dependencies(k7_serial_node
  rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs turtlesim k7_msgs
)
target_link_libraries(k7_serial_node k7_serial_lib)

install(TARGETS k7_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch udev
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
