cmake_minimum_required(VERSION 2.8.3)
project(exbotxi_avoidence)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES exbotxi_example
  CATKIN_DEPENDS geometry_msgs roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(avoidence_base nodes/avoidence_base.cpp)
target_link_libraries(avoidence_base ${catkin_LIBRARIES})

#add_library(csvparser src/csvparser.cpp)
#add_executable(avoidence_with_cmd nodes/avoidence_with_cmd.cpp)
#target_link_libraries(avoidence_with_cmd csvparser ${catkin_LIBRARIES})