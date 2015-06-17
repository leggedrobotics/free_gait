/*
 * BaseShiftDataRosWrapper.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_ros/BaseShiftDataRosWrapper.hpp>
#include <free_gait_ros/BaseShiftProfileRosWrapper.hpp>
#include <free_gait_ros/BaseShiftSplineTrajectoryRosWrapper.hpp>

// ROS
#include <ros/ros.h>

// STD
#include <string>

namespace free_gait {

BaseShiftDataRosWrapper::BaseShiftDataRosWrapper() :
    BaseShiftData()
{

}

BaseShiftDataRosWrapper::~BaseShiftDataRosWrapper()
{

}

bool BaseShiftDataRosWrapper::fromMessage(const quadruped_msgs::BaseShiftData& message)
{
  name_ = message.name;

  if (message.trajectory.joint_names.size() > 0) {
    // Trajectory.
    setUseProfile(false);
    BaseShiftSplineTrajectoryRosWrapper trajectory;
    if (!trajectory.fromMessage(message.trajectory)) return false;
    setTrajectory(trajectory);
  } else {
    setUseProfile(true);
    BaseShiftProfileRosWrapper profile;
    if (!profile.fromMessage(message.profile)) return false;
    setTrajectory(profile);
  }

  return true;
}

} /* namespace */
