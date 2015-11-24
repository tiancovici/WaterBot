cmake_minimum_required(VERSION 2.8.3)
project(explore_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
catkin_package()
add_executable(explore_node src/explore_node.cpp)

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${CMAKE_CXX_FLAGS}")

target_link_libraries(explore_node ${catkin_LIBRARIES})