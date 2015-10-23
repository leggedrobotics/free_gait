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
  virtual void initializeState(ExecutorState& executorState) const  = 0;
  virtual bool updateStateWithMeasurements(ExecutorState& executorState) const = 0;
  virtual bool updateExtras(const StepQueue& stepQueue, ExecutorState& executorState) const = 0;
};

} /* namespace free_gait */
