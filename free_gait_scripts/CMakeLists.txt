# Project configuration
cmake_minimum_required (VERSION 2.8)
project(free_gait_scripts)

add_definitions(--std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    quadruped_msgs
)

catkin_package(
  #INCLUDE_DIRS
  #LIBRARIES
  #CATKIN_DEPENDS
  #DEPENDS
)
