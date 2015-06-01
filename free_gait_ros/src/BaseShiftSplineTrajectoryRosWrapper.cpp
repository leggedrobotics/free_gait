/*
 * BaseShiftSplineTrajectoryRosWrapper.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_ros/BaseShiftSplineTrajectoryRosWrapper.hpp>
#include <ros/ros.h>

// STD
#include <string>

namespace free_gait {

BaseShiftSplineTrajectoryRosWrapper::BaseShiftSplineTrajectoryRosWrapper() :
    BaseShiftSplineTrajectory(),
    robot_utils_ros::RosMultiDOFJointTrajectoryInterface(),
    jointName_("base")
{

}

BaseShiftSplineTrajectoryRosWrapper::~BaseShiftSplineTrajectoryRosWrapper()
{

}

bool BaseShiftSplineTrajectoryRosWrapper::fromMessage(const trajectory_msgs::MultiDOFJointTrajectory& message)
{
  setFrameId(message.header.frame_id);
  robot_utils_ros::RosMultiDOFJointTrajectoryInterface trajectory;
  if (!trajectory.fromMessage(message, jointName_)) return false;
  trajectory_ = trajectory;
  return true;
}

} /* namespace */
