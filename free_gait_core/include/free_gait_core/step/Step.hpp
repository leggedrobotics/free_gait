/*
 * Step.hpp
 *
 *  Created on: Jan 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// STL
#include <string>
#include <unordered_map>
#include <iostream>
#include <memory>
#include <string>

// Loco
#include "loco/common/LegGroup.hpp"

// Free Gait
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/leg_motion/LegMotionBase.hpp"
#include "free_gait_core/base_motion/BaseMotionBase.hpp"

// Robot Utils
#include <robotUtils/containers/MultiKeyContainer.hpp>

// Quadruped model.
#include <quadruped_model/QuadrupedModel.hpp>

namespace free_gait {

class Step
{
 public:
  Step();
  virtual ~Step();

  /*!
   * Definition of the step states.
   */
  enum class State {Undefined, PreStep, AtStep, PostStep};

  /*!
   * Type definitions.
   */
  typedef std::unordered_map<quadruped_model::LimbEnum, LegMotionBase, robotUtils::EnumClassHash> LegMotions;
  typedef std::unordered_map<Step::State, BaseMotionBase, robotUtils::EnumClassHash> BaseMotions;

  /*!
   * Add step data (simplified input).
   * @param stepNumber the step number.
   * @param legName the name of the leg.
   * @param position the desired position of the step in world frame.
   */
//  void addSimpleStep(const int stepNumber, const std::string& legName, const free_gait::Position& position);

  /*!
   * Set the step number.
   * @param stepNumber the step number.
   */
  void setStepNumber(const int stepNumber);

  /*!
   * Add swing data for a leg.
   * @param legName the name of the leg.
   * @param data the step data.
   */
  void addLegMotion(const quadruped_model::LimbEnum& limb, const LegMotionBase& legMotion);

  /*!
   * Add base shift data for a state.
   * @param state the corresponding state of the base shift data.
   * @param data the base shift data.
   */
  void addBaseMotion(const Step::State& state, const BaseMotionBase& baseMotion);

  /*!
   * Checks if step has all data.
   * @return true if complete, false otherwise.
   */
  bool isComplete() const;

  /*!
   * Advance in time
   * @param dt the time step to advance [s].
   * @return true if step is active, false if finished.
   */
  bool advance(double dt);

  /*!
   * Checks status of robot to make sure if advancement is safe to continue.
   * @return true if true execution active, false if execution stopped
   *         because of unexpected event/state.
   */
  bool checkStatus();

  /*!
   * Get step number.
   * @return the step number.
   */
  unsigned int getStepNumber() const;

  /*!
   * Returns the current state (preStep, atStep, postStep) of the step.
   * @return the current state.
   */
  const Step::State& getState() const;

  /*!
   * Returns true if the state was switched in the last advancement update.
   * @return true if the state has switched, false otherwise.
   */
  bool hasSwitchedState() const;

  /*!
   * Returns the swing data as a map between leg name and swing data.
   * @return the swing data map.
   */
  Step::LegMotions& getLegMotions();

  BaseMotionBase& getCurrentBaseMotion();

  Step::BaseMotions& getBaseMotions();

  /*!
   * Return the current time of the step, starting at 0.0 for each step.
   * @return the current time.
   */
  double getTime() const;

  bool hasLegMotion() const;

  bool hasLegMotion(const quadruped_model::LimbEnum& limb) const;

  bool hasBaseMotion(const Step::State& state) const;

  double getCurrentStateTime();

  double getStateTime(const Step::State& state);

  /*!
   * Get the current state duration.
   * @return the current state duration.
   */
  double getCurrentStateDuration();

  double getStateDuration(const Step::State& state);

  double getAtStepDurationForLeg(const quadruped_model::LimbEnum& limb) const;

  double getTotalDuration();

  /*!
   * Get the current state phase.
   * @return the current state phase.
   */
  double getCurrentStatePhase();

  double getStatePhase(const Step::State& state);

  double getAtStepPhaseForLeg(const quadruped_model::LimbEnum& limb);

  double getTotalPhase();

  bool isApproachingEndOfState();

  friend std::ostream& operator << (std::ostream& out, const Step& step);

  friend class StepCompleter;

 protected:
  bool isComplete_;
  LegMotions legMotions_;
  BaseMotions baseMotions_;

 private:

  bool computeDurations();

  //! Current time, starts at 0.0 at each step.
  double time_;

  //! Current phase, starts at PreStep for each step.
  Step::State state_;

  //! State from the previous advancement.
  Step::State previousState_;

  //! Status from previous advancement.
  bool previousStatus_;

  double totalDuration_;
  double atStepDuration_;
  bool isDurationComputed_;

};

Step::State& operator++(Step::State& phase);
std::ostream& operator<<(std::ostream& os, const Step::State& phase);

} /* namespace */
