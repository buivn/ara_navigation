cmake_minimum_required(VERSION 2.8.3)
project(climb3_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  rospy
  serial
  std_msgs
)
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(control  src/main.cpp)
target_link_libraries(control ${catkin_LIBRARIES})