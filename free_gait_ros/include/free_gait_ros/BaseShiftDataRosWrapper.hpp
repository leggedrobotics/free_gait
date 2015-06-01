/*
 * BaseShiftDataRosWrapper.hpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/free_gait_core.hpp"

// ROS
#include <quadruped_msgs/BaseShiftData.h>

// STD
#include <string>

namespace free_gait {

class BaseShiftDataRosWrapper : public BaseShiftData
{
 public:
  BaseShiftDataRosWrapper();
  virtual ~BaseShiftDataRosWrapper();

  bool fromMessage(const quadruped_msgs::BaseShiftData& message);
};

} /* namespace */
