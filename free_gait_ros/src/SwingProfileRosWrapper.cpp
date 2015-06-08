/*
 * SwingProfileRosWrapper.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_ros/SwingProfileRosWrapper.hpp>
#include <ros/ros.h>

// STD
#include <string>

namespace free_gait {

SwingProfileRosWrapper::SwingProfileRosWrapper()
    : SwingProfile()
{

}

SwingProfileRosWrapper::~SwingProfileRosWrapper()
{

}

bool SwingProfileRosWrapper::fromMessage(const quadruped_msgs::SwingProfile& message)
{
  // Target frame.
  setFrameId(message.target.header.frame_id);

  // Target position.
  const auto& target = message.target.point;
  target_.x() = target.x;
  target_.y() = target.y;
  target_.z() = target.z;

  // Swing height.
  if (message.height != 0.0) {
    height_ = message.height;
  }

  // Duration.
  const auto& duration = ros::Duration(message.duration).toSec();
  if (duration != 0.0) {
    duration_ = duration;
  }

  // Type.
  if (message.type != "") {
    type_ = message.type;
  }

  return true;
}

} /* namespace */
