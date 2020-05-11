cmake_minimum_required(VERSION 2.8.3)
project(ars408_srv)

find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  message_generation
)


add_service_files(
  FILES
  Filter.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)