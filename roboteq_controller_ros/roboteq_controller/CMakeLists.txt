cmake_minimum_required(VERSION 3.5)
project(roboteq_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(Boost 1.71 CONFIG REQUIRED COMPONENTS thread)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(roboteq_interfaces REQUIRED)
find_package(serial REQUIRED)
find_package(turtlesim REQUIRED)


#include_directories(include)
#include_directories(/tmp/usr/local/include/)
#include_directories(/opt/ros/humble/include/)

set(node_plugins "")



add_executable(roboteq_controller_node src/roboteq_controller_node.cpp)
ament_target_dependencies(
  roboteq_controller_node serial
  rclcpp
  sensor_msgs
  std_msgs
  roboteq_interfaces
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(roboteq_ethernet_controller_node src/roboteq_ethernet_controller_node.cpp)
ament_target_dependencies(
  roboteq_ethernet_controller_node serial
  rclcpp
  sensor_msgs
  std_msgs
  roboteq_interfaces
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

#ament_target_dependencies(roboteq_controller_node rclcpp std_msgs roboteq_interfaces serial geometry_msgs)

add_executable(roboteq_tank_teleop src/roboteq_tank_teleop.cpp)
ament_target_dependencies(
  roboteq_tank_teleop serial
  rclcpp
  sensor_msgs
  std_msgs
  roboteq_interfaces
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  turtlesim
)

target_include_directories(roboteq_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_include_directories(roboteq_ethernet_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(TARGETS
  roboteq_controller_node
  roboteq_ethernet_controller_node
  roboteq_tank_teleop
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
