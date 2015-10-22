/*
 * StepExecutorAdapterBase.hpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_core/executor/ExecutorState.hpp"
#include "free_gait_core/step/Step.hpp"
#include <memory>

namespace free_gait {

class ExecutorAdapterBase
{
 public:
  ExecutorAdapterBase(std::shared_ptr<ExecutorState> state);
  virtual ~ExecutorAdapterBase();

  std::shared_ptr<ExecutorState> getState();

  virtual void updateFullRobotState() = 0;
  virtual void updateNonControlledRobotState(const Step& step) = 0;

 private:
  std::shared_ptr<ExecutorState> state_;

};

} /* namespace free_gait */
