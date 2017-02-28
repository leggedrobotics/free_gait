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
#include <free_gait_msgs/ExecuteStepsGoal.h>
#include <free_gait_msgs/Step.h>
#include <free_gait_msgs/Footstep.h>
#include <free_gait_msgs/LegMode.h>
#include <free_gait_msgs/JointTrajectory.h>
#include <free_gait_msgs/BaseAuto.h>
#include <free_gait_msgs/BaseTarget.h>
#include <free_gait_msgs/BaseTrajectory.h>

// Quadruped model
#undef LOG
#include "quadruped_model/QuadrupedModel.hpp"

// STD
#include <vector>

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
  bool fromMessage(const std::vector<free_gait_msgs::Step>& message, std::vector<free_gait::Step>& steps);
  bool fromMessage(const free_gait_msgs::Step& message, Step& step);
  bool fromMessage(const free_gait_msgs::Footstep& message, Footstep& footstep);
  bool fromMessage(const free_gait_msgs::EndEffectorTarget& message, EndEffectorTarget& endEffectorTarget);
  bool fromMessage(const free_gait_msgs::EndEffectorTrajectory& message, EndEffectorTrajectory& endEffectorTrajectory);
  bool fromMessage(const free_gait_msgs::LegMode& message, LegMode& legMode);
  bool fromMessage(const free_gait_msgs::JointTrajectory& message, JointTrajectory& jointTrajectory);
  bool fromMessage(const free_gait_msgs::BaseAuto& message, BaseAuto& baseAuto);
  bool fromMessage(const free_gait_msgs::BaseTarget& message, BaseTarget& baseTarget);
  bool fromMessage(const free_gait_msgs::BaseTrajectory& message, BaseTrajectory& baseTrajectory);
  bool fromMessage(const free_gait_msgs::CustomCommand& message, CustomCommand& customCommand);

  bool toMessage(const StepQueue& stepQueue, free_gait_msgs::ExecuteStepsGoal::_steps_type& message);
  bool toMessage(const Step& step, free_gait_msgs::Step& message);
  bool toMessage(const Footstep& footstep, free_gait_msgs::Footstep& message);
  bool toMessage(const EndEffectorTrajectory& endEffectorTrajectory, free_gait_msgs::EndEffectorTrajectory& message);
  bool toMessage(const BaseAuto& baseAuto, free_gait_msgs::BaseAuto& message);
  bool toMessage(const BaseTrajectory& baseTrajectory, free_gait_msgs::BaseTrajectory& message);

 private:
  std::shared_ptr<AdapterBase> adapter_;
};

}
/* namespace */
