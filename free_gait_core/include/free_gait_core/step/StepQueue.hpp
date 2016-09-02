/*
 * StepQueue.hpp
 *
 *  Created on: Jan 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// STL
#include <deque>
#include <string>
#include <memory>

// Free Gait
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/step/Step.hpp"

namespace free_gait {

class Step;

class StepQueue
{
 public:
  StepQueue();
  virtual ~StepQueue();
  StepQueue(const StepQueue& other);

  /*!
   * Add a step to the queue.
   * @param step the step to be added.
   */
  void add(const Step& step);
  void add(const std::vector<Step> steps);

  /*!
   * Advance in time
   * @param dt the time step to advance [s].
   * @return true if successful, false otherwise.
   */
  bool advance(double dt);

  bool hasSwitchedStep() const;
  bool hasStartedStep() const;

  /*!
   * Queue is active if a step is available and the step is ready/updated.
   * @return true if step should be processed, false otherwise.
   */
  bool active() const;

  /*!
   * Checks if queue contain steps.
   * @return true if queue empty, false otherwise.
   */
  bool empty() const;

  /*!
   * Clear queue but keeps the currently running step.
   */
  void clearNextSteps();

  /*!
   * Clear entire queue including currently active step.
   */
  void clear();

  /*!
   * Returns the current step. Check empty() first!
   * @return the current step.
   */
  const Step& getCurrentStep() const;
  Step& getCurrentStep();
  void replaceCurrentStep(const Step& step);

  /*!
   * Returns the next step. Check if size() > 1 first!
   * @return the next step.
   */
  const Step& getNextStep() const;

  const std::deque<Step>& getQueue() const;

  /*!
   * Returns the previous step. Returns null pointer
   * if no previous step is available.
   * @return the previous step.
   */
  bool previousStepExists() const;
  const Step& getPreviousStep() const;

  /*!
   * Returns the number of steps in the queue.
   * @return the number of steps.
   */
  std::deque<Step>::size_type size() const;

 private:

  //! Queue of step data.
  std::deque<Step> queue_;
  std::unique_ptr<Step> previousStep_;
  bool active_;
  bool hasSwitchedStep_, hasStartedStep_;
};

} /* namespace */
