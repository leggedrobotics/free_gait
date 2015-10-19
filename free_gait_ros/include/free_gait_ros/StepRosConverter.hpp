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

// Quadruped model
#include "quadruped_model/QuadrupedModel.hpp"

namespace free_gait {

class StepRosConverter
{
 public:
  StepRosConverter(std::shared_ptr<quadruped_model::QuadrupedModel> quadrupedModel);
  virtual ~StepRosConverter();

  /*!
   * Converts a ROS free gait step message to a step object.
   * @param[in] message the step message.
   * @param[out] gridMap the step object to be initialized.
   * @return true if successful, false otherwise.
   */
  bool fromMessage(const free_gait_msgs::Step& message, free_gait::Step& step);

  bool fromMessage(const free_gait_msgs::FootTarget& message, FootTarget& footTarget);

 private:
  Step::State getStepStateFromString(const std::string& state);

  //! Robot model.
  std::shared_ptr<quadruped_model::QuadrupedModel> quadrupedModel_;
};

}
/* namespace */
