cmake_minimum_required(VERSION 3.0.2)
project(avero_ctrl)

find_package(catkin_simple REQUIRED COMPONENTS geometry_msgs std_msgs avero_msgs mav_msgs )

catkin_simple(ALL_DEPS_REQUIRED avero_msgs)
add_definitions(-std=c++17)
add_definitions(-DMODE_ROSLOG)

# Add allocation_functions.cpp as a library
cs_add_library(allocation_functions src/allocation_functions.cpp)

# Add your executables
cs_add_executable(test_trajectory_pub src/test_trajectory_pub.cpp)
cs_add_executable(AveroSimulationNode src/AveroSimulationNode.cpp)
cs_add_executable(AveroFlyNode src/AveroFlyNode.cpp)
cs_add_executable(AveroTestingNode src/AveroTestingNode.cpp)
cs_add_executable(AveroAllocationTest src/AveroAllocationTest.cpp)


# Link the library to your executables
target_link_libraries(AveroSimulationNode allocation_functions)
target_link_libraries(AveroFlyNode allocation_functions)
target_link_libraries(AveroTestingNode allocation_functions)
target_link_libraries(AveroAllocationTest allocation_functions)


# Add dependencies
add_dependencies(AveroSimulationNode ${catkin_EXPORTED_TARGETS})
add_dependencies(AveroFlyNode ${catkin_EXPORTED_TARGETS})

cs_install()
cs_export()
