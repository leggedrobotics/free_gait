/*
 * SwingDataRosWrapper.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_ros/SwingDataRosWrapper.hpp>
#include <free_gait_ros/SwingFootTrajectoryRosWrapper.hpp>
#include <free_gait_ros/SwingProfileRosWrapper.hpp>
#include <free_gait_ros/SwingJointTrajectoryRosWrapper.hpp>
#include "free_gait_core/free_gait_core.hpp"

// ROS
#include <ros/ros.h>

// STD
#include <string>

namespace free_gait {

SwingDataRosWrapper::SwingDataRosWrapper()
    : SwingData()
{
}

SwingDataRosWrapper::~SwingDataRosWrapper()
{
}

bool SwingDataRosWrapper::fromMessage(const free_gait_msgs::SwingData& message)
{
  // Name.
  name_ = message.name;

  // Surface normal frame id.
  setSurfaceNormalFrameId(message.surface_normal.header.frame_id);

  // Surface normal.
  const auto& normal = message.surface_normal.vector;
  surfaceNormal_.x() = normal.x;
  surfaceNormal_.y() = normal.y;
  surfaceNormal_.z() = normal.z;

  // No touchdown.
  setNoTouchdown(message.no_touchdown);

  if (message.foot_trajectory.joint_names.size() > 0) {
    // Foot trajectory.
    SwingFootTrajectoryRosWrapper trajectory;
    if (!trajectory.fromMessage(message.foot_trajectory, name_))
      return false;
    setTrajectory(trajectory);
  } else if (message.joint_trajectory.joint_names.size() > 0) {
    // Joint trajectory.
    SwingJointTrajectoryRosWrapper trajectory;
    if (!trajectory.fromMessage(message.joint_trajectory))
      return false;
    setTrajectory(trajectory);
  } else {
    // Profile.
    SwingProfileRosWrapper profile;
    if (!profile.fromMessage(message.profile))
      return false;
    setTrajectory(profile);
  }

  return true;
}

} /* namespace */
