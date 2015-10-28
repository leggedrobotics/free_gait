/*
 * Executor.hpp
 *
 *  Created on: Oct 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <free_gait_core/executor/AdapterBase.hpp>
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/executor/State.hpp"
#include "free_gait_core/step/StepQueue.hpp"
#include "free_gait_core/step/StepCompleter.hpp"

// STD
#include <memory>

namespace free_gait {

class Executor
{
 public:
  Executor(std::shared_ptr<StepCompleter> completer, std::shared_ptr<AdapterBase> adapter,
           std::shared_ptr<State> state);
  virtual ~Executor();

  bool initialize();
  bool isInitialized() const;
  /*!
   * Advance in time
   * @param dt the time step to advance [s].
   * @return true if successful, false otherwise.
   */
  bool advance(double dt);
  void reset();

  const StepQueue& getQueue() const;
  StepQueue& getQueue();
  const State& getState() const;
  const AdapterBase& getAdapter() const;

 private:
  bool updateStateWithMeasurements();
  bool writeIgnoreContact();
  bool writeSupportLegs();
  bool writeLegMotion();
  bool writeTorsoMotion();

  StepQueue queue_;
  std::shared_ptr<StepCompleter> completer_;
  std::shared_ptr<AdapterBase> adapter_;
  std::shared_ptr<State> state_;
  bool isInitialized_;
};

} /* namespace free_gait */
