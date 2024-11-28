cmake_minimum_required(VERSION 3.0.2)
project(avero_testing)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  catkin_simple
  roscpp
  rospy
  std_msgs
)
catkin_simple(ALL_DEPENDS_REQUIRED)
add_definitions(-std=c++17 -Wno-reorder -Wno-unknown-pragmas -Werror -Wno-error=deprecated-declarations)
add_definitions(-DMODE_ROSLOG)

cs_add_executable(avero_thrust_testing_node include/avero_thrust_testing.h src/avero_thrust_testing.cpp src/avero_thrust_testing.cc)

cs_install()
cs_export()