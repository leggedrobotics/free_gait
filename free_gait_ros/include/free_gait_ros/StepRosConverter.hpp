/*
 * StepRosConverter.hpp
 *
 *  Created on: Feb 24, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/free_gait_core.hpp"

// Loco
#include "loco/common/LegGroup.hpp"

// ROS
#include <free_gait_msgs/Step.h>

// STD
#include <string>

namespace free_gait {

class StepRosConverter
{
 public:
  StepRosConverter();
  virtual ~StepRosConverter();

  /*!
   * Converts a ROS free gait step message to a step object.
   * @param[in] message the step message.
   * @param[out] gridMap the step object to be initialized.
   * @return true if successful, false otherwise.
   */
  static bool fromMessage(const free_gait_msgs::Step& message, free_gait::Step& step);
};

} /* namespace */
