cmake_minimum_required(VERSION 3.0.2)
project(show_track_path)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  intera_core_msgs
  roscpp
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES show_track_path
  CATKIN_DEPENDS intera_core_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(show_track_path_node
  src/show_track_path_node.cpp

  src/track_path.cpp
  include/show_track_path/track_path.h
)
target_link_libraries(show_track_path_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
