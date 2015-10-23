/*
 * StepExecutorAdapterBase.hpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_core/executor/ExecutorState.hpp"
#include "free_gait_core/step/StepQueue.hpp"
#include <memory>

namespace free_gait {

class ExecutorAdapterBase
{
 public:
  ExecutorAdapterBase();
  virtual ~ExecutorAdapterBase();
  virtual void updateStateWithMeasurements(ExecutorState& executorState) = 0;
  virtual void updateExtras(const StepQueue& stepQueue) = 0;
};

} /* namespace free_gait */
