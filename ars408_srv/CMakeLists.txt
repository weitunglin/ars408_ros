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

catkin_package(
  CATKIN_DEPENDS std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(filter_client src/filter_client.cpp)
target_link_libraries(filter_client ${catkin_LIBRARIES})
add_dependencies(filter_client ${catkin_EXPORTED_TARGETS})
