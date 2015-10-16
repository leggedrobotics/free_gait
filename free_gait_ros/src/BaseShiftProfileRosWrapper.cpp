/*
 * BaseShiftProfileRosWrapper.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_ros/BaseShiftProfileRosWrapper.hpp>
#include <ros/ros.h>

// Kindr
#include <kindr/thirdparty/ros/RosEigen.hpp>

// STD
#include <string>

namespace free_gait {

BaseShiftProfileRosWrapper::BaseShiftProfileRosWrapper()
    : BaseShiftProfile()
{

}

BaseShiftProfileRosWrapper::~BaseShiftProfileRosWrapper()
{

}

bool BaseShiftProfileRosWrapper::fromMessage(const free_gait_msgs::BaseShiftProfile& message)
{
  // Target frame.
  setFrameId(message.target.header.frame_id);

  // Target position.
  const auto& targetMsg = message.target.pose;
  if (!(targetMsg.position.x == 0 && targetMsg.position.y == 0 && targetMsg.position.z == 0
      && targetMsg.orientation.x == 0 && targetMsg.orientation.y == 0
      && targetMsg.orientation.z == 0 && targetMsg.orientation.w == 0)) {
    Pose target;
    kindr::poses::eigen_impl::convertFromRosGeometryMsg(message.target.pose, target);
    setTarget(target);
  }

  // Base height.
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
    profileType_ = message.type;
  }

  return true;
}

} /* namespace */
