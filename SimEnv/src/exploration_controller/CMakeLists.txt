cmake_minimum_required(VERSION 3.0.2)
project(exploration_controller)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

catkin_package(
  CATKIN_DEPENDS std_msgs rospy sensor_msgs nav_msgs geometry_msgs cv_bridge image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
