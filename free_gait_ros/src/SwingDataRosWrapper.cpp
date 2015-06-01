/*
 * SwingDataRosWrapper.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */


#include <free_gait_ros/SwingDataRosWrapper.hpp>
#include <free_gait_ros/SwingProfileRosWrapper.hpp>
#include <free_gait_ros/SwingSplineTrajectoryRosWrapper.hpp>

// Free Gait
#include "free_gait_core/free_gait_core.hpp"

// ROS
#include <ros/ros.h>

// STD
#include <string>

namespace free_gait {

SwingDataRosWrapper::SwingDataRosWrapper() :
    SwingData(),
    frameId_("map")
{

}

SwingDataRosWrapper::~SwingDataRosWrapper()
{

}

bool SwingDataRosWrapper::fromMessage(const quadruped_msgs::SwingData& message)
{
  // Name.
  name_ = message.name;

  // Surface normal.
  const auto& normal = message.surface_normal.vector;
  if (!(normal.x == 0.0 && normal.y == 0.0 && normal.z == 0.0
      && message.surface_normal.header.frame_id == "")) {
    if (message.surface_normal.header.frame_id == frameId_) {
      surfaceNormal_.x() = normal.x;
      surfaceNormal_.y() = normal.y;
      surfaceNormal_.z() = normal.z;
    } else {
      ROS_ERROR_STREAM(
          "Invalid surface normal frame in step message: '"
              << message.surface_normal.header.frame_id << "'.");
      return false;
    }
  }

  if (message.trajectory.joint_names.size() > 0) {
    // Trajectory.
    SwingSplineTrajectoryRosWrapper trajectory;
    if (!trajectory.fromMessage(message.trajectory, name_)) return false;
    setTrajectory(trajectory);
    setUseProfile(false);
  } else {
    // Profile.
    SwingProfileRosWrapper profile;
    if (!profile.fromMessage(message.profile)) return false;
    setTrajectory(profile);
    setUseProfile(true);
  }

  return true;
}

} /* namespace */
