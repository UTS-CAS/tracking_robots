cmake_minimum_required(VERSION 2.8.3)
project(track_robot_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  Trigger.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
