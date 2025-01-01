cmake_minimum_required(VERSION 3.0.2)
project(remote_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  airsim_ros
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS airsim_ros geometry_msgs roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_twist_node src/teleop_twist_node.cpp)

target_link_libraries(teleop_twist_node
  ${catkin_LIBRARIES}
)
