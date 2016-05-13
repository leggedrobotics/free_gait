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

// ROS
#include <free_gait_msgs/Step.h>
#include <free_gait_msgs/Footstep.h>
#include <free_gait_msgs/LegMode.h>
#include <free_gait_msgs/JointTrajectory.h>
#include <free_gait_msgs/BaseAuto.h>
#include <free_gait_msgs/BaseTarget.h>
#include <free_gait_msgs/BaseTrajectory.h>

// Quadruped model
#include "quadruped_model/QuadrupedModel.hpp"

namespace free_gait {

class StepRosConverter
{
 public:
  StepRosConverter(std::shared_ptr<AdapterBase> adapter);
  virtual ~StepRosConverter();

  /*!
   * Converts a ROS free gait step message to a step object.
   * @param[in] message the step message.
   * @param[out] gridMap the step object to be initialized.
   * @return true if successful, false otherwise.
   */
  bool fromMessage(const free_gait_msgs::Step& message, Step& step);
  bool fromMessage(const free_gait_msgs::Footstep& message, Footstep& footstep);
  bool fromMessage(const free_gait_msgs::EndEffectorTarget& message, EndEffectorTarget& endEffectorTarget);
  bool fromMessage(const free_gait_msgs::EndEffectorTrajectory& message, EndEffectorTrajectory& endEffectorTrajectory);
  bool fromMessage(const free_gait_msgs::LegMode& message, LegMode& legMode);
  bool fromMessage(const free_gait_msgs::JointTrajectory& message, JointTrajectory& jointTrajectory);
  bool fromMessage(const free_gait_msgs::BaseAuto& message, BaseAuto& baseAuto);
  bool fromMessage(const free_gait_msgs::BaseTarget& message, BaseTarget& baseTarget);
  bool fromMessage(const free_gait_msgs::BaseTrajectory& message, BaseTrajectory& baseTrajectory);

 private:
  std::shared_ptr<AdapterBase> adapter_;
};

}
/* namespace */
