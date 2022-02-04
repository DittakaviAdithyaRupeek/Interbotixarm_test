cmake_minimum_required(VERSION 3.10.0)
project(auxiliary)
find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_python_setup()
catkin_package()