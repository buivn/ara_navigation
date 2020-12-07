cmake_minimum_required(VERSION 2.8.3)
project(ara_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joy
  message_generation
  roscpp
  rospy
  serial
  std_msgs
  geometry_msgs
  eigen_conversions
  actionlib
  actionlib_msgs
  nav_msgs
  dynamixel_workbench_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
  FILES DynamixelCommand.srv magarr_controller.srv nextPose_move.srv
 )

## Generate actions in the 'action' folder
add_action_files( DIRECTORY action
  FILES araControl.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   actionlib_msgs
   nav_msgs
   dynamixel_workbench_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES climb3
 CATKIN_DEPENDS joy message_generation roscpp rospy serial std_msgs eigen_conversions 
 geometry_msgs actionlib nav_msgs dynamixel_workbench_msgs
 # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/climb3.cpp
# )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_climb3.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})
add_dependencies(main ara_control_gencpp)

# add_executable(icra21 src/icra21.cpp)
# target_link_libraries(icra21 ${catkin_LIBRARIES})
# add_dependencies(icra21 ara_control_gencpp)

add_executable(main_icra21 src/main_icra21.cpp)
target_link_libraries(main_icra21 ${catkin_LIBRARIES})
add_dependencies(main_icra21 ara_control_gencpp)

add_executable(araControl src/araControl.cpp)
target_link_libraries(araControl ${catkin_LIBRARIES})
add_dependencies(araControl ara_control_gencpp ${catkin_EXPORTED_TARGETS})


add_executable(araClient src/araClient.cpp)
target_link_libraries(araClient ${catkin_LIBRARIES})
add_dependencies(araClient ara_control_gencpp)


add_executable(process_trajectory src/process_trajectory.cpp)
target_link_libraries(process_trajectory ${catkin_LIBRARIES})
add_dependencies(process_trajectory ara_control_gencpp)


# add_executable(clientNode src/clientNode_class.cpp)
# target_link_libraries(clientNode ${catkin_LIBRARIES})
# add_dependencies(clientNode ara_control_gencpp)

# add_executable(check_magarr src/check_magarr.cpp)
# target_link_libraries(check_magarr ${catkin_LIBRARIES})
# add_dependencies(check_magarr ara_control_gencpp)
