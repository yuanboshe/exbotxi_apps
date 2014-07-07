cmake_minimum_required(VERSION 2.8.3)
project(exbotxi_recognition)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  message_generation
)

add_message_files(
  FILES
  TwistWithMask.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES exbotxi_example
  CATKIN_DEPENDS geometry_msgs roscpp rospy message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(csvparser src/csvparser.cpp)

add_executable(cmd_center nodes/cmd_center.cpp)
add_dependencies(cmd_center ${PROJECT_NAME}_gencpp)
target_link_libraries(cmd_center csvparser ${catkin_LIBRARIES})

add_executable(vel_center nodes/vel_center.cpp)
target_link_libraries(vel_center ${catkin_LIBRARIES})