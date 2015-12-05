cmake_minimum_required(VERSION 2.8.3)
project(localizer_cpp_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  std_msgs
)
catkin_package()
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

##Search for all source files and headers

add_executable(localizer_cpp_node src/localizer_cpp_node.cpp src/particle.cpp src/mcl_tools.cpp src/raycaster.cpp)

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${CMAKE_CXX_FLAGS}")

target_link_libraries(localizer_cpp_node ${catkin_LIBRARIES})

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )
target_link_libraries(localizer_cpp_node ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(localizer_cpp_node ${OpenCV_LIBRARIES})
