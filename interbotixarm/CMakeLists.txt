cmake_minimum_required(VERSION 3.10.0)
project(interbotixarm)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  interbotix_xsarm_control
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Find JSONCPP Library and Headers
# https://github.com/open-source-parsers/jsoncpp/issues/927
find_package(jsoncpp REQUIRED)
get_target_property(JSONCPP_INCLUDE_DIR jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)

## Find MOSQUITTO Library and Headers
if (NOT MOSQUITTO_INCLUDE_DIR)
  find_path(MOSQUITTO_INCLUDE_DIR mosquittopp.h)
endif()

if (NOT MOSQUITTOCPP_LIBRARY)
  find_library(MOSQUITTOCPP_LIBRARY NAMES mosquittopp)
endif()

# Add custom message files and generate
add_message_files(
  FILES
  jsButton.msg
  JointGroupCommand.msg
)

#Add custom services in srv folder
add_service_files(
   FILES
   SleepPose.srv
   DesiredPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

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
  LIBRARIES mqttroswrapper
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime interbotix_xsarm_control
)



###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MOSQUITTO_INCLUDE_DIR}
  ${JSONCPP_INCLUDE_DIR}
)

## Declare a C++ library
add_library(mqttroswrapper
  src/mqttroswrapper.cpp
)
target_link_libraries(mqttroswrapper
  ${catkin_LIBRARIES}
  ${MOSQUITTOCPP_LIBRARY}
  jsoncpp_lib
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
add_executable(mqtt2ros_streamer_node src/mqtt2ros_streamer_node.cpp)
target_link_libraries(mqtt2ros_streamer_node
  ${catkin_LIBRARIES}
  mqttroswrapper
)
add_executable(dummy_joystick_node src/dummy_joystick_node.cpp)
target_link_libraries(dummy_joystick_node
  ${catkin_LIBRARIES}
  mqttroswrapper
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
# Python scripts for installation
catkin_install_python(PROGRAMS 
  scripts/joystick_control.py
  scripts/currentviz.py
  scripts/service_helper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark executables for installation
install(TARGETS mqtt2ros_streamer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS dummy_joystick_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
