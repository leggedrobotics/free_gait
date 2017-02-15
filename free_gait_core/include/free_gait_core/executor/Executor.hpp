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
#include <robot_utils/timers/ChronoTimer.hpp>

// STD
#include <memory>
#include <string>

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
  void pause(bool shouldPause);

  /*!
   * Stop the current steps. Depending on the preemption type,
   * this method returns `true` if action was actually stopped,
   * or `false` if it cannot be stopped.
   * @return true if action is stopped, false otherwise.
   */
  bool stop();

  void addToFeedback(const std::string& feedbackDescription);
  const std::string& getFeedbackDescription() const;
  void clearFeedbackDescription();

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

  enum class PreemptionType {
    PREEMPT_IMMEDIATE,
    PREEMPT_STEP,
    PREEMPT_NO
  };

 private:
  bool completeCurrentStep(bool multiThreaded = false);
  bool resetStateWithRobot();
  bool updateStateWithMeasurements();
  bool writeIgnoreContact();
  bool writeIgnoreForPoseAdaptation();
  bool writeSupportLegs();
  bool writeSurfaceNormals();
  bool writeLegMotion();
  bool writeTorsoMotion();

  Mutex mutex_;
  bool isInitialized_;
  bool isPausing_;
  PreemptionType preemptionType_;
  StepQueue queue_;
  std::shared_ptr<StepCompleter> completer_;
  std::shared_ptr<StepComputer> computer_;
  std::shared_ptr<AdapterBase> adapter_;
  std::shared_ptr<State> state_;
  std::string feedbackDescription_;
  bool firstFeedbackDescription_;
};

} /* namespace free_gait */
