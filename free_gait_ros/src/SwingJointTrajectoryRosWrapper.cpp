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
    const trajectory_msgs::JointTrajectory& message, const std::string& jointName)
{
//  size_t nJoints = message.joint_names.size();
//  size_t j = 0;
//  for (; j < nJoints; ++j) {
//    if (message.joint_names[j] == jointName) break;
//    if (j == nJoints - 1) return false; // Joint name not found.
//  }
//
//  for (const auto& point : message.points) {
//    times_.push_back(ros::Duration(point.time_from_start).toSec());
//    Position position;
//    kindr::phys_quant::eigen_impl::convertFromRosGeometryMsg(point.transforms[j].translation, position);
//    values_.push_back(position.vector());
//  }

  return true;
}

} /* namespace */
