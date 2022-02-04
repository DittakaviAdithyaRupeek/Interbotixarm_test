cmake_minimum_required(VERSION 3.10.0)
project(kdlarm)

## Find used packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    kdl_parser
)
find_package (orocos_kdl REQUIRED)

## Declare a catkin package
catkin_package()

## Build our node
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
)

## Executables and linking
add_executable(main src/kdlArm.cpp src/node.cpp)
target_link_libraries(main
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
)
