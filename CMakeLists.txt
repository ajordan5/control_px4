cmake_minimum_required(VERSION 3.0.2)
project(control_px4)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)