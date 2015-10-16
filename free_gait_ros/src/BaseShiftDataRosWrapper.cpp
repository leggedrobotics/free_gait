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

bool BaseShiftDataRosWrapper::fromMessage(const free_gait_msgs::BaseShiftData& message)
{
  name_ = message.name;
  ignore_ = message.ignore;
  if (ignore_) return true;

  std::string type(message.type);
  if (type.empty()) {
    // Try to figure out what user meant.
    if (message.trajectory.joint_names.size() > 0) {
      type = "trajectory";
    } else {
      type = "profile";
    }
  }

  if (type == "trajectory") {
    BaseShiftSplineTrajectoryRosWrapper trajectory;
    if (!trajectory.fromMessage(message.trajectory)) return false;
    setTrajectory(trajectory);
  } else if (type == "profile") {
    BaseShiftProfileRosWrapper profile;
    if (!profile.fromMessage(message.profile)) return false;
    setTrajectory(profile);
  } else {
    return false;
  }

  return true;
}

} /* namespace */
