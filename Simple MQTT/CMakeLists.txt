cmake_minimum_required(VERSION 3.12)
project(remote_assaying_machine VERSION 1.0.0 LANGUAGES CXX)

if (NOT MOSQUITTO_INCLUDE_DIR)
  find_path(MOSQUITTO_INCLUDE_DIR mosquittopp.h)
endif()

if (NOT MOSQUITTOCPP_LIBRARY)
  find_library(MOSQUITTOCPP_LIBRARY NAMES mosquittopp)
endif()

# get header files for library compilation
include_directories(
  include
  ${MOSQUITTO_INCLUDE_DIR}
)

# list all source files in src
set(SOURCES          
    src/mqttwrapper.cpp
)

# build a library out of the src files
add_library(mqttwrapper SHARED ${SOURCES})

# external libraries needed by the custom built library
target_link_libraries(mqttwrapper PUBLIC ${MOSQUITTOCPP_LIBRARY})

# create our executable programs 
add_executable(publisher app/publish.cpp)
target_link_libraries(publisher PRIVATE mqttwrapper)
add_executable(subscriber app/subscribe.cpp)
target_link_libraries(subscriber PRIVATE mqttwrapper)

set_target_properties(
    mqttwrapper publisher subscriber 
      PROPERTIES 
        CXX_STANDARD 11
        CXX_STANDARD_REQUIRED YES 
        CXX_EXTENSIONS NO
)