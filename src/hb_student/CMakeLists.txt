cmake_minimum_required(VERSION 2.8.3)
project(hb_student)

find_package(catkin REQUIRED COMPONENTS
  rospy
  hb_common
  hb_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy hb_common hb_msgs
)
