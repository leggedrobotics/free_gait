/*
 * Executor.hpp
 *
 *  Created on: Oct 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <free_gait_core/executor/ExecutorAdapterBase.hpp>
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/executor/ExecutorState.hpp"
#include "free_gait_core/step/StepQueue.hpp"
#include "free_gait_core/step/StepCompleter.hpp"

// STD
#include <memory>

namespace free_gait {

class Executor
{
 public:
  Executor(std::shared_ptr<StepCompleter> completer, std::shared_ptr<ExecutorAdapterBase> adapter,
           std::shared_ptr<ExecutorState> state);
  virtual ~Executor();

  /*!
   * Advance in time
   * @param dt the time step to advance [s].
   * @return true if successful, false otherwise.
   */
  bool advance(double dt);
  void reset();

  const StepQueue& getQueue() const;
  const ExecutorState& getState() const;

 private:
  bool updateStateWithMeasurements();
  bool writeIgnoreContact();
  bool writeSupportLegs();

  StepQueue queue_;
  std::shared_ptr<StepCompleter> completer_;
  std::shared_ptr<ExecutorAdapterBase> adapter_;
  std::shared_ptr<ExecutorState> state_;
};

} /* namespace free_gait */
