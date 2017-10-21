/*
 * Footstep.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/leg_motion/Footstep.hpp>
#include <free_gait_core/leg_motion/LegMotionBase.hpp>

// Message logger
#include <message_logger/message_logger.hpp>

namespace free_gait {

Footstep::Footstep(LimbEnum limb)
    : EndEffectorMotionBase(LegMotionBase::Type::Footstep, limb),
      profileHeight_(0.0),
      averageVelocity_(0.0),
      liftOffSpeed_(0.0),
      touchdownSpeed_(0.0),
      duration_(0.0),
      minimumDuration_(0.0),
      ignoreContact_(false),
      ignoreForPoseAdaptation_(false),
      isComputed_(false),
      controlSetup_ { {ControlLevel::Position, true}, {ControlLevel::Velocity, true},
                      {ControlLevel::Acceleration, true}, {ControlLevel::Effort, false} }
{
}

Footstep::~Footstep()
{
}

std::unique_ptr<LegMotionBase> Footstep::clone() const
{
  std::unique_ptr<LegMotionBase> pointer(new Footstep(*this));
  return pointer;
}

const ControlSetup Footstep::getControlSetup() const
{
  return controlSetup_;
}

void Footstep::updateStartPosition(const Position& startPosition)
{
  isComputed_ = false;
  start_ = startPosition;
}

void Footstep::updateStartVelocity(const LinearVelocity& startVelocity)
{
 // Nothing to todo.
}

const Position Footstep::getStartPosition() const
{
  return start_;
}

const LinearVelocity Footstep::getStartVelocity() const
{
  return liftOffVelocity_;
}

bool Footstep::compute(bool isSupportLeg)
{
  values_.clear();
  if (profileType_ == "triangle") {
    generateTriangleKnots();
  } else if (profileType_ == "square") {
    generateSquareKnots();
  } else if (profileType_ == "straight") {
    generateStraightKnots();
  } else if (profileType_ == "trapezoid") {
    generateTrapezoidKnots();
  } else {
    MELO_ERROR_STREAM("Swing profile of type '" << profileType_ << "' not supported.");
    return false;
  }

  times_.clear();
  computeTiming(values_, averageVelocity_, minimumDuration_, times_);
  std::vector<DerivativeType> velocities, accelerations;
  if (ignoreContact_) touchdownSpeed_ = 0.0;
  if (!isSupportLeg) liftOffSpeed_ = 0.0;
  Vector surfaceNormal;
  if (surfaceNormal_) {
    surfaceNormal = *surfaceNormal_;
  } else {
    surfaceNormal =  Vector::UnitZ();
  }
  liftOffVelocity_ = LinearVelocity(liftOffSpeed_ * Vector::UnitZ());
  touchdownVelocity_ = LinearVelocity(-touchdownSpeed_ * surfaceNormal.vector());
  trajectory_.clear();
  trajectory_.fitCurveWithDerivatives(times_, values_, liftOffVelocity_.vector(), touchdownVelocity_.vector());
  duration_ = trajectory_.getMaxTime() - trajectory_.getMinTime();
  isComputed_ = true;
  return true;
}

bool Footstep::prepareComputation(const State& state, const Step& step, const AdapterBase& adapter)
{
  return compute(state.isSupportLeg(limb_));
}

bool Footstep::needsComputation() const
{
  return false;
}

bool Footstep::isComputed() const
{
  return isComputed_;
}

void Footstep::reset()
{
  start_.setZero();
  trajectory_.clear();
  duration_ = 0.0;
  isComputed_ = false;
}

const Position Footstep::evaluatePosition(const double time) const
{
  const double timeInRange = mapTimeWithinDuration(time);
  Position position;
  trajectory_.evaluate(position.toImplementation(), timeInRange);
  return position;
}

const LinearVelocity Footstep::evaluateVelocity(const double time) const
{
  const double timeInRange = mapTimeWithinDuration(time);
  LinearVelocity velocity;
  trajectory_.evaluateDerivative(velocity.toImplementation(), timeInRange, 1);
  return velocity;
}

const LinearAcceleration Footstep::evaluateAcceleration(const double time) const
{
  const double timeInRange = mapTimeWithinDuration(time);
  LinearAcceleration acceleration;
  trajectory_.evaluateDerivative(acceleration.toImplementation(), timeInRange, 2);
  return acceleration;
}

double Footstep::getDuration() const
{
  return duration_;
}

void Footstep::setMinimumDuration(const double minimumDuration)
{
  minimumDuration_ = minimumDuration;
  isComputed_ = false;
}

double Footstep::getMinimumDuration() const
{
  return minimumDuration_;
}

void Footstep::setTargetPosition(const std::string& frameId, const Position& target)
{
  frameId_ = frameId;
  target_ = target;
}

const Position Footstep::getTargetPosition() const
{
  return target_;
}

const LinearVelocity Footstep::getTargetVelocity() const
{
  return touchdownVelocity_;
}

const std::string& Footstep::getFrameId(const ControlLevel& controlLevel) const
{
  if (controlLevel == ControlLevel::Effort) {
    throw std::runtime_error("Footstep::getFrameId() is only valid for position or velocity.");
  }
  return frameId_;
}

void Footstep::setProfileType(const std::string& profileType)
{
  profileType_ = profileType;
}

const std::string& Footstep::getProfileType() const
{
  return profileType_;
}

void Footstep::setProfileHeight(const double profileHeight)
{
  profileHeight_ = profileHeight;
}

double Footstep::getProfileHeight() const
{
  return profileHeight_;
}

double Footstep::getAverageVelocity() const
{
  return averageVelocity_;
}

void Footstep::setAverageVelocity(double averageVelocity)
{
  averageVelocity_ = averageVelocity;
}

bool Footstep::isIgnoreContact() const
{
  return ignoreContact_;
}

bool Footstep::isIgnoreForPoseAdaptation() const
{
  return ignoreForPoseAdaptation_;
}

const std::vector<Footstep::ValueType>& Footstep::getKnotValues() const
{
  return values_;
}

const std::vector<Footstep::Time>& Footstep::getTimes() const
{
  return times_;
}

std::ostream& operator<<(std::ostream& out, const Footstep& footstep)
{
  out << "Frame: " << footstep.getFrameId(ControlLevel::Position) << std::endl;
  out << "Height: " << footstep.profileHeight_ << std::endl;
  out << "Average velocity: " << footstep.averageVelocity_ << std::endl;
  out << "Type: " << footstep.profileType_ << std::endl;
  out << "Start Position: " << footstep.start_ << std::endl;
  out << "Target Position: " << footstep.target_ << std::endl;
  out << "Duration: " << footstep.getDuration() << std::endl;
  out << "Lift-Off Speed: " << footstep.liftOffSpeed_ << std::endl;
  out << "Touchdown Speed: " << footstep.touchdownSpeed_ << std::endl;
  return out;
}

void Footstep::generateStraightKnots()
{
  // Knot 1.
  values_.push_back(start_.vector());

  // Knot 2.
  values_.push_back(target_.vector());
}

void Footstep::generateTriangleKnots()
{
  // Knot 1.
  values_.push_back(start_.vector());

  // Knot 2.
  // Interpolate on the xy-plane.
  Position knot2 = start_ + 0.5 * (target_ - start_);
  // Apex height.
  double basis = start_.z() > target_.z() ? start_.z() : target_.z();
  knot2.z() = basis + profileHeight_;
  values_.push_back(knot2.vector());

  // Knot 3.
  values_.push_back(target_.vector());
}

void Footstep::generateSquareKnots()
{
  double basis = start_.z() > target_.z() ? start_.z() : target_.z();
  double height = basis + profileHeight_;

  // Knot 1.
  values_.push_back(start_.vector());

  // Knot 2.
  Position knot2(start_.x(), start_.y(), height);
  values_.push_back(knot2.vector());

  // Knot 3.
  Position knot3(target_.x(), target_.y(), height);
  values_.push_back(knot3.vector());

  // Knot 4.
  values_.push_back(target_.vector());
}

void Footstep::generateTrapezoidKnots()
{
  // Knot 1.
  values_.push_back(start_.vector());

  // Knot 2.
  Position knot2 = start_ + 0.1 * (target_ - start_);
  knot2.z() = start_.z() + profileHeight_;
  values_.push_back(knot2.vector());

  // Knot 4.
  Position knot4 = start_ + 0.9 * (target_ - start_);
  knot4.z() = target_.z() + profileHeight_;

  // Knot 3.
  Position knot3 = knot2 + 0.5 * (knot4 - knot2);
  knot3.z() = knot4.z();
  values_.push_back(knot3.vector());
  values_.push_back(knot4.vector());

  // Knot 6.
  values_.push_back(target_.vector());
}

void Footstep::computeTiming(const std::vector<ValueType>& values, const double averageVelocity, double minimumDuration,
                             std::vector<Time>& times)
{
  times.clear();
  times.push_back(0.0);
  for (size_t i = 1; i < values.size(); ++i) {
    double distance = (values[i] - values[i-1]).norm();
    double duration = distance / averageVelocity;
    times.push_back(times[i-1] + duration);
  }
  if (times.back() < minimumDuration) {
    for (size_t i = 1; i < times.size(); ++i) {
      times[i] = times[i] / times.back() * minimumDuration;
    }
  }
}

} /* namespace */
