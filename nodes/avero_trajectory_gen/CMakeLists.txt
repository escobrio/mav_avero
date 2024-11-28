cmake_minimum_required(VERSION 3.0.2)
project(avero_trajectory_gen)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

############
# BINARIES #
############
cs_add_executable(trajectory_planner
        src/planner.cpp
        src/planner_node.cpp
)

cs_add_executable(trajectory_to_odom
        src/trajectory_to_odom.cpp
)

##########
# EXPORT #
##########
cs_install()
cs_export()
