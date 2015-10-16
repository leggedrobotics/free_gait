/*
 * SwingJointTrajectoryRosWrapper.cpp
 *
 *  Created on: Sep 2, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_ros/SwingJointTrajectoryRosWrapper.hpp>
#include <ros/ros.h>

// STD
#include <string>

// Curves
#include <curves_ros/RosJointTrajectoryInterface.hpp>

namespace free_gait {

SwingJointTrajectoryRosWrapper::SwingJointTrajectoryRosWrapper() :
    SwingJointTrajectory()
{
}

SwingJointTrajectoryRosWrapper::~SwingJointTrajectoryRosWrapper()
{
}

bool SwingJointTrajectoryRosWrapper::fromMessage(
    const trajectory_msgs::JointTrajectory& message)
{
  size_t nJoints = message.joint_names.size();
  size_t j = 0;
  for (const auto& point : message.points) {
    times_.push_back(ros::Duration(point.time_from_start).toSec());
  }
  for (; j < nJoints; ++j) {
    values_.push_back(std::vector<ValueType>());
    for (const auto& point : message.points) {
      values_[j].push_back(point.positions[j]);
    }
  }
  return true;
}

} /* namespace */
