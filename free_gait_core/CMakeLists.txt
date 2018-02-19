cmake_minimum_required(VERSION 2.8.3)
project(free_gait_core)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  quadruped_model
  numopt_common
  numopt_quadprog
  numopt_sqp
  curves
  grid_map_core
  robot_utils
  std_utils
  message_logger
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS
      include
      ${EIGEN3_INCLUDE_DIR}
    LIBRARIES
      ${PROJECT_NAME}
    CATKIN_DEPENDS
      quadruped_model
      numopt_common
      numopt_quadprog
      numopt_sqp
      curves
      grid_map_core
      robot_utils
      std_utils
      message_logger
)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
   src/TypeDefs.cpp
   src/TypePrints.cpp
   src/base_motion/BaseMotionBase.cpp
   src/base_motion/BaseAuto.cpp
   src/base_motion/BaseTarget.cpp
   src/base_motion/BaseTrajectory.cpp
   src/leg_motion/LegMotionBase.cpp
   src/leg_motion/EndEffectorMotionBase.cpp
   src/leg_motion/JointMotionBase.cpp
   src/leg_motion/LegMode.cpp
   src/leg_motion/JointTrajectory.cpp
   src/leg_motion/EndEffectorTrajectory.cpp
   src/leg_motion/EndEffectorTarget.cpp
   src/leg_motion/Footstep.cpp
   src/step/Step.cpp
   src/step/StepQueue.cpp
   src/step/StepCompleter.cpp
   src/step/StepComputer.cpp
   src/step/CustomCommand.cpp
   src/executor/Executor.cpp
   src/executor/ExecutorState.cpp
   src/executor/BatchExecutor.cpp
   src/executor/State.cpp
   src/executor/StateBatch.cpp
   src/executor/StateBatchComputer.cpp
   src/executor/AdapterBase.cpp
   src/pose_optimization/PoseOptimizationBase.cpp
   src/pose_optimization/PoseConstraintsChecker.cpp
   src/pose_optimization/PoseOptimizationGeometric.cpp
   src/pose_optimization/PoseOptimizationQP.cpp
   src/pose_optimization/PoseOptimizationSQP.cpp
   src/pose_optimization/PoseParameterization.cpp
   src/pose_optimization/PoseOptimizationObjectiveFunction.cpp
   src/pose_optimization/PoseOptimizationFunctionConstraints.cpp
   src/pose_optimization/PoseOptimizationProblem.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
# Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/test_free_gait_core.cpp
  test/AdapterDummy.cpp
  test/StepTest.cpp
  test/FootstepTest.cpp
#  test/PoseOptimizationQpTest.cpp
#  test/PoseOptimizationSQPTest.cpp
)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
  )
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
