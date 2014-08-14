cmake_minimum_required(VERSION 2.8.3)
project(strands_hostname)

find_package(catkin REQUIRED COMPONENTS
  rospy
  scitos_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
