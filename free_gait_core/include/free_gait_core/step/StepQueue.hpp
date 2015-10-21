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

class StepQueue
{
 public:
  StepQueue();
  virtual ~StepQueue();

  /*!
   * Add a step to the queue.
   * @param step the step to be added.
   */
  void add(std::shared_ptr<Step> step);

  /*!
   * Advance in time
   * @param dt the time step to advance [s].
   * @return true if successful, false otherwise.
   */
  bool advance(double dt);

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
  std::shared_ptr<Step> getCurrentStep();

  /*!
   * Returns the next step. Check if size() > 1 first!
   * @return the next step.
   */
  std::shared_ptr<Step> getNextStep();

  /*!
   * Returns the previous step. Returns null pointer
   * if no previous step is available.
   * @return the previous step.
   */
  std::shared_ptr<Step> getPreviousStep();

  /*!
   * Returns the number of steps in the queue.
   * @return the number of steps.
   */
  std::deque<std::shared_ptr<Step>>::size_type size() const;

  /*!
   * Return true if the step was changed in the last advance() update.
   * @return true if step was switched, false otherwise.
   */
  bool hasSwitchedStep() const;

 private:

  //! Queue of step data.
  std::deque<std::shared_ptr<Step>> queue_;

  std::shared_ptr<Step> previousStep_;

  bool hasSwitchedStep_;
};

} /* namespace */
