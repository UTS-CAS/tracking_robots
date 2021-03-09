cmake_minimum_required(VERSION 3.0.2)
project(track_robot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  intera_core_msgs
  intera_motion_msgs
  roscpp
  sensor_msgs
  rosbag
  std_msgs
  track_robot_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS intera_core_msgs intera_motion_msgs roscpp sensor_msgs rosbag std_msgs track_robot_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(track_robot_node
  src/track_robot_node.cpp

  src/track_robot.cpp
  include/track_robot/track_robot.h
  )
target_link_libraries(track_robot_node
  ${catkin_LIBRARIES}
  )
