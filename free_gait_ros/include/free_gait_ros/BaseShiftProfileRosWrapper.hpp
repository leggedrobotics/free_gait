/*
 * BaseShiftProfileRosWrapper.hpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/free_gait_core.hpp"

// ROS
#include <quadruped_msgs/BaseShiftProfile.h>

// STD
#include <string>

namespace free_gait {

class BaseShiftProfileRosWrapper : public BaseShiftProfile
{
 public:
  BaseShiftProfileRosWrapper();
  virtual ~BaseShiftProfileRosWrapper();

  bool fromMessage(const quadruped_msgs::BaseShiftProfile& message);

 private:
  const std::string frameId_;
};

} /* namespace */
