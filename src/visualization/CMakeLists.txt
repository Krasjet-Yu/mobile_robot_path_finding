cmake_minimum_required(VERSION 3.0.2)
project(visualization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES visualization
 CATKIN_DEPENDS roscpp rospy std_msgs visualization_msgs
 DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
