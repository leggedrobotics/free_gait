# Project configuration
cmake_minimum_required (VERSION 2.8)
project(loco_joint_trajectories_ros)

add_definitions(--std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    loco_ros
    loco_joint_trajectories
    robot_utils_ros
    actionlib
    loco_free_gait_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS loco_joint_trajectories loco_ros robot_utils_ros loco_free_gait_ros
#  DEPENDS system_lib
)

find_package(Eigen3 REQUIRED)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${kindr_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
   src/JointTrajectoriesRos.cpp
   src/RosControllerInterfaceJointTrajectories.cpp
)

add_dependencies(${PROJECT_NAME}
    loco_ros
    loco_joint_trajectories
    robot_utils_ros
    loco_free_gait_ros
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
