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

// Robot utils
#include <robotUtils/timers/ChronoTimer.hpp>

// STD
#include <memory>
#include <thread>

// Boost
#include <boost/thread.hpp>

namespace free_gait {

class Executor
{
 public:
  Executor(std::shared_ptr<StepCompleter> completer, std::shared_ptr<AdapterBase> adapter,
           std::shared_ptr<State> state);
  virtual ~Executor();

  /*!
   * Type definitions.
   */
  typedef boost::shared_mutex Mutex;
  typedef boost::shared_lock<boost::shared_mutex> SharedLock;
  typedef boost::unique_lock<boost::shared_mutex> UniqueLock;

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

  /*!
   * Note: To make usage of this method thread-safe, use the lock on mutex
   * of this class provided by the getMutex() method.
   * @return
   */
  StepQueue& getQueue();
  Mutex& getQueueMutex();

  const State& getState() const;
  Mutex& getStateMutex();

  const AdapterBase& getAdapter() const;
  Mutex& getAdapterMutex();

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

  Mutex adapterMutex_;
  Mutex stateMutex_;
  Mutex queueMutex_;
  bool isInitialized_;
  StepQueue queue_;
  std::shared_ptr<StepCompleter> completer_;
  std::shared_ptr<AdapterBase> adapter_;
  std::shared_ptr<State> state_;
};

} /* namespace free_gait */
