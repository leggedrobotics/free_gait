/*
 * Step.cpp
 *
 *  Created on: Jan 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "free_gait_core/step/Step.hpp"
#include "free_gait_core/base_motion/base_motion.hpp"
#include "free_gait_core/TypeDefs.hpp"

// Roco
#include <roco/log/log_messages.hpp>

namespace free_gait {

inline void boundToRange(double* v, double min, double max){
  if (*v < min) *v = min;
  if (*v > max) *v = max;
}

inline double mapTo01Range(double v, double min, double max){
  double t = v;
  if (fabs(min - max) < 0.000000001) return 1;
  boundToRange(&t, min, max);
  t = (t-min)/(max-min);
  return t;
}

Step::Step()
    : time_(0.0),
      isComplete_(false),
      totalDuration_(0.0),
      isUpdated_(false)
{
  legMotions_.clear();
  baseMotion_.reset();
}

Step::~Step()
{
}

Step::Step(const Step& other) :
    time_(other.time_),
    isComplete_(other.isComplete_),
    totalDuration_(other.totalDuration_),
    isUpdated_(other.isUpdated_)
{
  if (other.baseMotion_) baseMotion_ = std::move(other.baseMotion_->clone());
  legMotions_.clear();
  for (const auto& legMotion : other.legMotions_) {
    legMotions_[legMotion.first] = std::move(legMotion.second->clone());
  }
}

Step& Step::operator=(const Step& other)
{
  isComplete_ = other.isComplete_;
  time_ = other.time_;
  totalDuration_ = other.time_;
  isUpdated_ = other.isUpdated_;
  if (other.baseMotion_) baseMotion_ = std::move(other.baseMotion_->clone());
  legMotions_.clear();
  for (const auto& legMotion : other.legMotions_) {
    legMotions_[legMotion.first] = std::move(legMotion.second->clone());
  }
  return *this;
}

std::unique_ptr<Step> Step::clone() const
{
  std::unique_ptr<Step> pointer(new Step(*this));
  return pointer;
}

void Step::addLegMotion(const LimbEnum& limb, const LegMotionBase& legMotion)
{
  legMotions_.insert(std::pair<LimbEnum, std::unique_ptr<LegMotionBase>>(limb, std::move(legMotion.clone())));
  isUpdated_ = false;
  isComplete_ = false;
}

void Step::addBaseMotion(const BaseMotionBase& baseMotion)
{
  baseMotion_ = std::move(baseMotion.clone());
  isUpdated_ = false;
  isComplete_ = false;
}

bool Step::isUpdated() const
{
  return isUpdated_;
}

bool Step::update()
{
  if (!isComplete_) {
    for (const auto& legMotion : legMotions_) {
      if (!legMotion.second->isComputed()) return false;
    }
    if (hasBaseMotion()) {
      if (!baseMotion_->isComputed()) return false;
    }
  }
  isComplete_ = true;

  totalDuration_ = 0.0;
  for (const auto& legMotion : legMotions_) {
    if (legMotion.second->getDuration() > totalDuration_)
      totalDuration_ = legMotion.second->getDuration();
  }
  if (hasBaseMotion()) {
    if (baseMotion_->getDuration() > totalDuration_)
      totalDuration_ = baseMotion_->getDuration();
  }
  return isUpdated_ = true;
}

bool Step::advance(double dt)
{
  if (!isUpdated_) throw std::runtime_error("Step::advance() cannot be called if step is not updated.");
  time_ += dt;
  if (time_ > getTotalDuration()) return false;
  return true;
}

bool Step::hasLegMotion() const
{
  return !legMotions_.empty();
}

bool Step::hasLegMotion(const LimbEnum& limb) const
{
  return !(legMotions_.find(limb) == legMotions_.end());
}

const LegMotionBase& Step::getLegMotion(const LimbEnum& limb) const
{
  if (!hasLegMotion(limb)) throw std::out_of_range("No leg motion for this limb in this step!");
  return *legMotions_.at(limb);
}

bool Step::hasBaseMotion() const
{
  return (bool)(baseMotion_);
}

const BaseMotionBase& Step::getBaseMotion() const
{
  if (!hasBaseMotion()) throw std::out_of_range("No base motion in this step!");
  return *baseMotion_;
}

double Step::getTime() const
{
  return time_;
}

//double Step::getStateTime(const Step::State& state)
//{
//  switch (state) {
//    case State::PreStep:
//      return getTime();
//    case State::AtStep:
//      return getTime() - getStateDuration(State::PreStep);
//    case State::PostStep:
//      return getTime() - getStateDuration(State::PreStep) - getStateDuration(State::AtStep);
//    default:
//      return 0.0;
//  }
//}

double Step::getTotalDuration() const
{
  if (!isUpdated_) throw std::runtime_error("Step::getTotalDuration() cannot be called if step is not updated.");
  return totalDuration_;
}

double Step::getTotalPhase() const
{
  if (!isUpdated_) throw std::runtime_error("Step::getTotalPhase() cannot be called if step is not updated.");
  return mapTo01Range(time_, 0.0, getTotalDuration());
}

//double Step::getTotalDuration()
//{
////  switch (state) {
////    case Step::State::PreStep:
////      return hasBaseMotion(State::PreStep) ? baseMotions_.at(State::PreStep).getDuration() : 0.0;
////    case Step::State::AtStep:
////      if (!isDurationComputed_) computeDurations();
////      return atStepDuration_;
////    case Step::State::PostStep:
////      return hasBaseMotion(State::PostStep) ? baseMotions_.at(State::PostStep).getDuration() : 0.0;
////    default:
////      return 0.0;
////  }
//}

double Step::getLegMotionDuration(const LimbEnum& limb) const
{
  if (!isUpdated_) throw std::runtime_error("Step::getLegMotionDuration() cannot be called if step is not updated.");
  if (!hasLegMotion(limb)) return 0.0;
  return legMotions_.at(limb)->getDuration();
}

double Step::getLegMotionPhase(const LimbEnum& limb) const
{
  if (!isUpdated_) throw std::runtime_error("Step::getLegMotionPhase() cannot be called if step is not updated.");
  return mapTo01Range(time_, 0.0, getLegMotionDuration(limb));
}

double Step::getBaseMotionDuration() const
{
  if (!isUpdated_) throw std::runtime_error("Step::getBaseMotionDuration() cannot be called if step is not updated.");
  if (!hasBaseMotion()) return 0.0;
  return baseMotion_->getDuration();
}

double Step::getBaseMotionPhase() const
{
  if (!isUpdated_) throw std::runtime_error("Step::getBaseMotionPhase() cannot be called if step is not updated.");
  return mapTo01Range(time_, 0.0, getBaseMotionDuration());
}

//
//double Step::getStatePhase(const Step::State& state)
//{
//  switch (state) {
//    case Step::State::PreStep:
//      return loco::mapTo01Range(getTime(), 0.0, getStateDuration(State::PreStep));
//    case Step::State::AtStep:
//      return loco::mapTo01Range(getTime() - getStateDuration(State::PreStep), 0.0, getStateDuration(State::AtStep));
//    case Step::State::PostStep:
//      return loco::mapTo01Range(getTime() - getStateDuration(State::PreStep) - getStateDuration(State::AtStep), 0.0,
//                                getStateDuration(State::PostStep));
//    default:
//      return 0.0;
//  }
//}

//double Step::getAtStepPhaseForLeg(const quadruped_model::LimbEnum& limb)
//{
//  return loco::mapTo01Range(getTime() - getStateDuration(State::PreStep), 0.0, getAtStepDurationForLeg(limb));
//}
//
//double Step::getTotalPhase()
//{
//  return loco::mapTo01Range(getTime(), 0.0, getTotalDuration());
//}

bool Step::isApproachingEnd(double tolerance) const
{
  if (!isUpdated_) throw std::runtime_error("Step::isApproachingEnd() cannot be called if step is not updated.");
  if (getTime() + tolerance >= getTotalDuration()) return true;
  return false;
}

std::ostream& operator<<(std::ostream& out, const Step& step)
{
  if (step.hasLegMotion()) {
    out << "---" << std::endl;
    out << "Leg motions (" << step.legMotions_.size() << "):" << std::endl;
    for (const auto& legMotion : step.legMotions_) out << "-" << std::endl << *(legMotion.second) << std::endl;
  }
  if (step.hasBaseMotion()) {
    out << "---" << std::endl;
    out << "Base motion: " << std::endl;
    out << *(step.baseMotion_) << std::endl;
  }
  return out;
}

} /* namespace */
