cmake_minimum_required(VERSION 2.8.3)
project(ars408_msg)

find_package(catkin REQUIRED
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bbox.msg
  Bboxes.msg
  GPSinfo.msg
  pathPoint.msg
  pathPoints.msg
  RadarPoint.msg
  RadarPoints.msg
  Object.msg
  Objects.msg
  Usonic.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs
)
