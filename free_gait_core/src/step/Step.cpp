/*
 * Step.cpp
 *
 *  Created on: Jan 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "free_gait_core/step/Step.hpp"

// Roco
#include <roco/log/log_messages.hpp>

// Loco
#include "loco/state_switcher/StateSwitcher.hpp"
#include "loco/utils/math.hpp"

namespace free_gait {

Step::Step()
    : time_(0.0),
      state_(Step::State::Undefined),
      previousState_(Step::State::Undefined),
      previousStatus_(false),
      isComplete_(false),
      totalDuration_(NAN),
      atStepDuration_(NAN),
      isDurationComputed_(false)
{

}

Step::~Step()
{
}

//void Step::addSimpleStep(const int stepNumber, const std::string& legName,
//                       const loco::Position& target)
//{
//  stepNumber_ = stepNumber;
//  SwingData swingData;
//  swingData.setName(legName);
//  SwingProfile profile;
//  profile.setTarget(target);
//  swingData.setTrajectory(profile);
//  legMotion_.insert(std::pair<std::string, SwingData>(legName, swingData));
//  isDurationComputed_ = false;
//}

void Step::addLegMotion(const quadruped_model::LimbEnum& limb, const LegMotionBase& legMotion)
{
  legMotions_.insert(std::pair<quadruped_model::LimbEnum, LegMotionBase>(limb, legMotion));
  isDurationComputed_ = false;
}

void Step::addBaseMotion(const Step::State& state, const BaseMotionBase& baseMotion)
{
  baseMotions_.insert(std::pair<Step::State, BaseMotionBase>(state, baseMotion));
  isDurationComputed_ = false;
}

bool Step::isComplete() const
{
  return isComplete_;
}

bool Step::advance(double dt)
{
  if (!isComplete())
    return false;  // TODO This is not ok.

  bool status = checkStatus();
  if (!status) {
    ROCO_WARN_THROTTLE_STREAM(1.0, "Not continuing step. Status not ok.");
    return true;
  }

  if (status && !previousStatus_) {
    ROCO_DEBUG_STREAM("Continuing with step.");
  }

  previousStatus_ = status;
  previousState_ = state_;

  time_ += dt;
  if (time_ >= getTotalDuration())
    return false;
  if (time_ >= getStateDuration(State::PreStep) + getStateDuration(State::AtStep)) {
    state_ = Step::State::PostStep;
  } else if (time_ >= getStateDuration(State::PreStep)) {
    state_ = Step::State::AtStep;
  } else {
    state_ = Step::State::PreStep;
  }

  return true;
}

bool Step::checkStatus()
{
//  for (auto leg : *legs_) {
//    loco::StateSwitcher* stateSwitcher = leg->getStateSwitcher();
//    switch (stateSwitcher->getState()) {
//      case (loco::StateSwitcher::States::StanceSlipping):
//      case (loco::StateSwitcher::States::StanceLostContact):
//      case (loco::StateSwitcher::States::SwingExpectingContact):
//        ROCO_WARN_THROTTLE_STREAM(1.0, "Leg " << leg->getName() << " should be grounded but it is not.");
//        return false;
//
//      case (loco::StateSwitcher::States::StanceNormal):
//      case (loco::StateSwitcher::States::SwingNormal):
//      case (loco::StateSwitcher::States::SwingLateLiftOff):
//      case (loco::StateSwitcher::States::SwingBumpedIntoObstacle):
//      case (loco::StateSwitcher::States::SwingEarlyTouchDown):
//      case (loco::StateSwitcher::States::Init):
//        break;
//
//      default:
//        ROCO_ERROR_STREAM_FP("Unhandled state: " << stateSwitcher->getStateName(stateSwitcher->getState()) << std::endl);
//        return false;
//    }
//  }

  return true;
}

const Step::State& Step::getState() const
{
  return state_;
}

bool Step::hasSwitchedState() const
{
  return (previousState_ != state_);
}

Step::LegMotions& Step::getLegMotions()
{
  return legMotions_;
}

BaseMotionBase& Step::getCurrentBaseMotion()
{
  if (!hasBaseMotion(state_)) throw std::out_of_range("No base motion for current state!");
  return baseMotions_.at(state_);
}

Step::BaseMotions& Step::getBaseMotions()
{
  return baseMotions_;
}

double Step::getTime() const
{
  return time_;
}

bool Step::hasLegMotion() const
{
  return !legMotions_.empty();
}

bool Step::hasLegMotion(const quadruped_model::LimbEnum& limb) const
{
  return !(legMotions_.find(limb) == legMotions_.end());
}

bool Step::hasBaseMotion(const Step::State& state) const
{
  return !(baseMotions_.find(state) == baseMotions_.end());
}

double Step::getCurrentStateTime()
{
  return getStateTime(getState());
}

double Step::getStateTime(const Step::State& state)
{
  switch (state) {
    case State::PreStep:
      return getTime();
    case State::AtStep:
      return getTime() - getStateDuration(State::PreStep);
    case State::PostStep:
      return getTime() - getStateDuration(State::PreStep) - getStateDuration(State::AtStep);
    default:
      return 0.0;
  }
}

double Step::getCurrentStateDuration()
{
  return getStateDuration(getState());
}

double Step::getStateDuration(const Step::State& state)
{
  switch (state) {
    case Step::State::PreStep:
      return hasBaseMotion(State::PreStep) ? baseMotions_.at(State::PreStep).getDuration() : 0.0;
    case Step::State::AtStep:
      if (!isDurationComputed_) computeDurations();
      return atStepDuration_;
    case Step::State::PostStep:
      return hasBaseMotion(State::PostStep) ? baseMotions_.at(State::PostStep).getDuration() : 0.0;
    default:
      return 0.0;
  }
}

double Step::getAtStepDurationForLeg(const quadruped_model::LimbEnum& limb) const
{
  if (!hasLegMotion(limb)) return 0.0;
  return legMotions_.at(limb).getDuration();
}

double Step::getTotalDuration()
{
  if (!isDurationComputed_)
    computeDurations();
  return totalDuration_;
}

double Step::getCurrentStatePhase()
{
  return getStatePhase(getState());
}

double Step::getStatePhase(const Step::State& state)
{
  switch (state) {
    case Step::State::PreStep:
      return loco::mapTo01Range(getTime(), 0.0, getStateDuration(State::PreStep));
    case Step::State::AtStep:
      return loco::mapTo01Range(getTime() - getStateDuration(State::PreStep), 0.0, getStateDuration(State::AtStep));
    case Step::State::PostStep:
      return loco::mapTo01Range(getTime() - getStateDuration(State::PreStep) - getStateDuration(State::AtStep), 0.0,
                                getStateDuration(State::PostStep));
    default:
      return 0.0;
  }
}

double Step::getAtStepPhaseForLeg(const quadruped_model::LimbEnum& limb)
{
  return loco::mapTo01Range(getTime() - getStateDuration(State::PreStep), 0.0, getAtStepDurationForLeg(limb));
}

double Step::getTotalPhase()
{
  return loco::mapTo01Range(getTime(), 0.0, getTotalDuration());
}

bool Step::isApproachingEndOfStep()
{
  double tolerance = 0.01;
  if (getTime() + tolerance >= getTotalDuration()) return true;
  return false;
}

bool Step::isApproachingEndOfState()
{
  double tolerance = 0.01;
  if (getCurrentStateTime() + tolerance >= getCurrentStateDuration()) return true;
  return false;
}

bool Step::computeDurations()
{
  if (!isComplete())
    return false;

  double maxAtStepDuration = 0.0;
  for (const auto& legMotion : legMotions_) {
    if (legMotion.second.getDuration() > maxAtStepDuration)
      maxAtStepDuration = legMotion.second.getDuration();
  }
  if (hasBaseMotion(State::AtStep)) {
    if (baseMotions_.at(State::AtStep).getDuration() > maxAtStepDuration)
      maxAtStepDuration = baseMotions_.at(State::AtStep).getDuration();
  }
  atStepDuration_ = maxAtStepDuration;
  totalDuration_ = getStateDuration(State::PreStep) + getStateDuration(State::AtStep) + getStateDuration(State::PostStep);
  return isDurationComputed_ = true;
}

std::ostream& operator<<(std::ostream& out, const Step& step)
{
  out << "Leg motion: " << std::endl;
  for (const auto& legMotion : step.legMotions_) out << legMotion.second << std::endl;
  out << "Base motion: " << std::endl;
  for (const auto& baseMotion : step.baseMotions_) out << baseMotion.second << std::endl;
  return out;
}

Step::State& operator++(Step::State& phase)
{
  switch (phase) {
    case Step::State::PreStep:
      return phase = Step::State::AtStep;
    case Step::State::AtStep:
      return phase = Step::State::PostStep;
    case Step::State::PostStep:
      return phase = Step::State::PostStep;
  }
}

std::ostream& operator<<(std::ostream& os, const Step::State& phase)
{
  switch (phase) {
    case Step::State::Undefined:
          os << "State::Undefined";
          return os;
    case Step::State::PreStep:
      os << "State::PreStep";
      return os;
    case Step::State::AtStep:
      os << "State::AtStep";
      return os;
    case Step::State::PostStep:
      os << "State::PostStep";
      return os;
  }
}

} /* namespace */
