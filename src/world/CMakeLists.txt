cmake_minimum_required(VERSION 2.8.3)
project(world)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)
catkin_package(
)

include_directories(

  ${catkin_INCLUDE_DIRS}
)
