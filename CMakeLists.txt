cmake_minimum_required(VERSION 3.0.2)
project(control_px4)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  std_msgs
)

find_package(ublox QUIET)

if (${ublox_FOUND})
   message(WARNING "found ublox library")
   add_definitions(-DUBLOX) #defines UBLOX
else()
   message(WARNING "Did NOT find ublox library")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
)