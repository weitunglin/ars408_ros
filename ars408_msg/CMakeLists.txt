cmake_minimum_required(VERSION 2.8.3)
project(ars408_msg)

find_package(catkin REQUIRED
  std_msgs
  sensor_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bbox.msg
  Bboxes.msg
  Motion.msg
  pathPoint.msg
  pathPoints.msg
  RadarPoint.msg
  RadarPoints.msg
  Object.msg
  Objects.msg
  Usonic.msg
  BatchImage.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs
)
