/*
 * StepRosWrapper.hpp
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
#include "loco/common/TorsoBase.hpp"

// ROS
#include <quadruped_msgs/Step.h>

// STD
#include <string>

namespace free_gait {

class StepRosWrapper : public Step
{
 public:
  StepRosWrapper(std::shared_ptr<loco::LegGroup> legs, std::shared_ptr<loco::TorsoBase> torso);
  virtual ~StepRosWrapper();

  /*!
   * Populate step from ROS step data message.
   * @param message the step data ROS message.
   */
  bool fromMessage(const quadruped_msgs::Step& message);
};

} /* namespace */
