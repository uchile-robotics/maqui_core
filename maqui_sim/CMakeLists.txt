cmake_minimum_required(VERSION 2.8.3)
project(maqui_sim)

find_package(catkin REQUIRED COMPONENTS
  pepper_gazebo_plugin
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES maqui_sim
#  CATKIN_DEPENDS pepper_gazebo_plugin
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
