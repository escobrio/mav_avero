cmake_minimum_required(VERSION 3.0.2)
project(avero_joy)

find_package(catkin_simple REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_simple()

cs_add_executable(joy_to_odom src/joy_to_odom.cpp)
cs_add_executable(joy_to_wrench src/joy_to_wrench.cpp)

cs_install()
cs_export()