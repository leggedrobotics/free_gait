/*
 * SwingSplineTrajectoryRosWrapper.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_ros/SwingSplineTrajectoryRosWrapper.hpp>
#include <ros/ros.h>

// STD
#include <string>

// Kindr
#include "kindr/thirdparty/ros/RosGeometryMsgPhysicalQuantitiesEigen.hpp"

namespace free_gait {

SwingSplineTrajectoryRosWrapper::SwingSplineTrajectoryRosWrapper() :
    SwingSplineTrajectory()
{

}

SwingSplineTrajectoryRosWrapper::~SwingSplineTrajectoryRosWrapper()
{

}

bool SwingSplineTrajectoryRosWrapper::fromMessage(
    const trajectory_msgs::MultiDOFJointTrajectory& message, const std::string& jointName)
{
  setFrameId(message.header.frame_id);

  size_t nJoints = message.joint_names.size();
  size_t j = 0;
  for (; j < nJoints; ++j) {
    if (message.joint_names[j] == jointName) break;
    if (j == nJoints - 1) return false; // Joint name not found.
  }

  for (const auto& point : message.points) {
    times_.push_back(ros::Duration(point.time_from_start).toSec());
    Position position;
    kindr::phys_quant::eigen_impl::convertFromRosGeometryMsg(point.transforms[j].translation, position);
    values_.push_back(position.vector());
  }

  return true;
}

} /* namespace */
