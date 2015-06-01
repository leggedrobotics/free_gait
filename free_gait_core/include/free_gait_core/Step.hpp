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
#include <map>
#include <unordered_map>
#include <iostream>
#include <memory>
#include <string>

// Loco
#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoBase.hpp"

// Free Gait
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/SwingData.hpp"
#include "free_gait_core/BaseShiftData.hpp"

namespace free_gait {

class Step
{
 public:
  Step(std::shared_ptr<loco::LegGroup> legs, std::shared_ptr<loco::TorsoBase> torso);
  virtual ~Step();

  /*!
   * Definition of the step states.
   */
  enum class State {Undefined, PreStep, AtStep, PostStep};

  /*!
   * Add step data (simplified input).
   * @param stepNumber the step number.
   * @param legName the name of the leg.
   * @param position the desired position of the step in world frame.
   */
  void addSimpleStep(const int stepNumber, const std::string& legName, const free_gait::Position& position);

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
  void addSwingData(const std::string& legName, const SwingData& data);

  /*!
   * Add base shift data for a state.
   * @param state the corresponding state of the base shift data.
   * @param data the base shift data.
   */
  void addBaseShiftData(const Step::State& state, const BaseShiftData& data);

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
  std::unordered_map<std::string, SwingData>& getSwingData();

  BaseShiftData& getCurrentBaseShiftData();

  std::map<Step::State, BaseShiftData>& getBaseShiftData();

  /*!
   * Return the current time of the step, starting at 0.0 for each step.
   * @return the current time.
   */
  double getTime() const;

  bool hasSwingData() const;

  bool hasSwingData(const std::string& legName) const;

  bool hasBaseShiftData(const Step::State& state) const;

  /*!
   * Get the current state duration.
   * @return the current state duration.
   */
  double getStateDuration();

  double getStateTime();

  double getStateTime(const Step::State& state);

  /*!
   * Get the current state phase.
   * @return the current state phase.
   */
  double getStatePhase();

  double getPreStepDuration() const;

  double getPreStepPhase() const;

  double getAtStepDuration();

  double getAtStepDuration(const std::string& legName) const;

  double getAtStepPhase();
  double getAtStepPhase(const std::string& legName);

  double getPostStepDuration() const;

  double getPostStepPhase();

  double getTotalDuration();

  double getTotalPhase();

  friend std::ostream& operator << (std::ostream& out, const Step& step);

  friend class StepCompleter;

 protected:
  bool isComplete_;
  std::unordered_map<std::string, SwingData> swingData_;
  std::map<State, BaseShiftData> baseShiftData_;

 private:

  bool computeDurations();

  //! Common loco leg group class.
  std::shared_ptr<loco::LegGroup> legs_;

  //! Common loco torso class.
  std::shared_ptr<loco::TorsoBase> torso_;

  //! Step number.
  unsigned int stepNumber_;

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

} /* namespace loco */
