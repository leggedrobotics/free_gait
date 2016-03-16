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
#include "free_gait_core/step/StepComputer.hpp"

// Robot utils
#include <robotUtils/timers/ChronoTimer.hpp>

// STD
#include <memory>

// Boost
#include <boost/thread/recursive_mutex.hpp>

namespace free_gait {

class Executor
{
 public:
  Executor(std::shared_ptr<StepCompleter> completer, std::shared_ptr<StepComputer> computer,
           std::shared_ptr<AdapterBase> adapter, std::shared_ptr<State> state);
  virtual ~Executor();

  /*!
   * Type definitions.
   */
  typedef boost::recursive_mutex Mutex;
  typedef boost::recursive_mutex::scoped_lock Lock;

  bool initialize();
  bool isInitialized() const;

  Mutex& getMutex();

  /*!
   * Advance in time
   * @param dt the time step to advance [s].
   * @return true if successful, false otherwise.
   */
  bool advance(double dt);

  void reset();

  const StepQueue& getQueue() const;

  /*!
   * Note: To make usage of this method thread-safe, use the lock on mutex
   * of this class provided by the getMutex() method.
   * @return
   */
  StepQueue& getQueue();
  const State& getState() const;
  const AdapterBase& getAdapter() const;

 private:
  bool completeCurrentStep(bool multiThreaded = false);
  bool initializeStateWithRobot();
  bool updateStateWithMeasurements();
  bool writeIgnoreContact();
  bool writeIgnoreForPoseAdaptation();
  bool writeSupportLegs();
  bool writeSurfaceNormals();
  bool writeLegMotion();
  bool writeTorsoMotion();

  Mutex mutex_;
  bool isInitialized_;
  StepQueue queue_;
  std::shared_ptr<StepCompleter> completer_;
  std::shared_ptr<StepComputer> computer_;
  std::shared_ptr<AdapterBase> adapter_;
  std::shared_ptr<State> state_;
};

} /* namespace free_gait */
