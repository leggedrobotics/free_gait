/*
 * BaseShiftProfileRosWrapper.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_ros/BaseShiftProfileRosWrapper.hpp>
#include <ros/ros.h>

// STD
#include <string>

namespace free_gait {

BaseShiftProfileRosWrapper::BaseShiftProfileRosWrapper() :
    BaseShiftProfile(),
    frameId_("map")
{

}

BaseShiftProfileRosWrapper::~BaseShiftProfileRosWrapper()
{

}

bool BaseShiftProfileRosWrapper::fromMessage(const quadruped_msgs::BaseShiftProfile& message)
{
    // Target position.
    const auto& target = message.target.pose;
    if (!(target.position.x == 0.0 && target.position.y == 0.0 && target.position.z == 0.0
        && message.target.header.frame_id == "")) {
      if (message.target.header.frame_id == frameId_) {
        target_.getPosition().x() = target.position.x;
        target_.getPosition().y() = target.position.y;
        target_.getPosition().z() = target.position.z;
      } else {
        ROS_ERROR_STREAM(
            "Invalid target position frame in step message: '" << message.target.header.frame_id << "'.");
        return false;
      }
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
      type_ = message.type;
    }

    return true;
}

} /* namespace */
