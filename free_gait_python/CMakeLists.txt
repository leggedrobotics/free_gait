# Project configuration
cmake_minimum_required (VERSION 2.8)
project(free_gait_python)

add_definitions(--std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    free_gait_msgs
    locomotion_controller_msgs
)

catkin_package(
  #INCLUDE_DIRS
  #LIBRARIES
  #CATKIN_DEPENDS
  #DEPENDS
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  bin/free_gait/action_loader.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
