/*
 * FootTarget.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/leg_motion/FootTarget.hpp"
#include "free_gait_core/leg_motion/LegMotionBase.hpp"

// Roco
#include <roco/log/log_messages.hpp>

namespace free_gait {

FootTarget::FootTarget()
    : LegMotionBase(LegMotionBase::Type::FootTarget),
      profileHeight_(0.0),
      averageVelocity_(0.0),
      trajectoryUpdated_(false)
{
}

FootTarget::~FootTarget()
{
}

std::unique_ptr<LegMotionBase> FootTarget::clone() const
{
  std::unique_ptr<LegMotionBase> pointer(new FootTarget(*this));
  return pointer;
}

bool FootTarget::updateStartPosition(const Position& startPosition)
{
  start_ = startPosition;
  trajectoryUpdated_ = false;
  return computeTrajectory();
}

const Position FootTarget::evaluate(const double phase)
{
  if (!trajectoryUpdated_) computeTrajectory();
  const double time = phase * getDuration();
  return Position(trajectory_.evaluate(time));
}

double FootTarget::getDuration() const
{
  return averageVelocity_;
}

double FootTarget::getAverageVelocity() const
{
  return averageVelocity_;
}

void FootTarget::setAverageVelocity(double averageVelocity)
{
  averageVelocity_ = averageVelocity;
  trajectoryUpdated_ = false;
}

double FootTarget::getProfileHeight() const
{
  return profileHeight_;
}

void FootTarget::setProfileHeight(double profileHeight)
{
  profileHeight_ = profileHeight;
  trajectoryUpdated_ = false;
}

const Position FootTarget::getTarget() const
{
  return target_;
}

void FootTarget::setTarget(const Position& target)
{
  target_ = target;
  trajectoryUpdated_ = false;
}

const std::string& FootTarget::getFrameId() const
{
  return frameId_;
}

void FootTarget::setFrameId(const std::string& frameId)
{
  frameId_ = frameId;
}

const std::string& FootTarget::getProfileType() const
{
  return profileType_;
}

void FootTarget::setProfileType(const std::string& type)
{
  profileType_ = type;
  trajectoryUpdated_ = false;
}

const Vector& FootTarget::getSurfaceNormal() const
{
  return surfaceNormal_;
}

void FootTarget::setSurfaceNormal(const Vector& surfaceNormal)
{
  surfaceNormal_ = surfaceNormal;
}

bool FootTarget::isNoTouchdown() const
{
  return noTouchdown_;
}

void FootTarget::setNoTouchdown(bool noTouchdown)
{
  noTouchdown_ = noTouchdown;
}


bool FootTarget::isIgnoreForPoseAdaptation() const
{
  return ignoreForPoseAdaptation_;
}

void FootTarget::setIgnoreForPoseAdaptation(bool ignoreForPoseAdaptation)
{
  ignoreForPoseAdaptation_ = ignoreForPoseAdaptation;
}

std::ostream& operator<<(std::ostream& out, const FootTarget& footTarget)
{
  out << "Target: " << footTarget.target_ << std::endl;
  out << "Height: " << footTarget.profileHeight_ << std::endl;
  out << "Duration: " << footTarget.averageVelocity_ << std::endl;
  out << "Type: " << footTarget.profileType_;
  return out;
}

bool FootTarget::computeTrajectory()
{
  std::vector<Time> times;
  std::vector<ValueType> values;

  if (profileType_ == "triangle") {
    generateTriangleKnots(times, values);
  } else if (profileType_ == "square") {
    generateSquareKnots(times, values);
  } else if (profileType_ == "straight") {
    generateStraightKnots(times, values);
  } else {
    ROCO_ERROR_STREAM("Swing profile of type '" << profileType_ << "' not supported.");
    return false;
  }

  trajectory_.fitCurve(times, values);
  trajectoryUpdated_ = true;
  return true;
}

void FootTarget::generateStraightKnots(std::vector<Time>& times,
                                         std::vector<ValueType>& values) const
{
  // Knot 1.
  times.push_back(0.0);
  values.push_back(start_.vector());

  // Knot 2.
  times.push_back(averageVelocity_);
  values.push_back(target_.vector());
}

void FootTarget::generateTriangleKnots(std::vector<Time>& times,
                                         std::vector<ValueType>& values) const
{
  // Knot 1.
  times.push_back(0.0);
  values.push_back(start_.vector());

  // Knot 2.
  times.push_back(0.5 * averageVelocity_);
  // Interpolate on the xy-plane.
  Position knot2 = start_ + 0.5 * (target_ - start_);
  // Apex height.
  double basis = start_.z() > target_.z() ? start_.z() : target_.z();
  knot2.z() = basis + profileHeight_;
  values.push_back(knot2.vector());

  // Knot 3.
  times.push_back(averageVelocity_);
  values.push_back(target_.vector());
}

void FootTarget::generateSquareKnots(std::vector<Time>& times,
                                       std::vector<ValueType>& values) const
{
  double basis = start_.z() > target_.z() ? start_.z() : target_.z();
  double height = basis + profileHeight_;

  // Knot 1.
  times.push_back(0.0);
  values.push_back(start_.vector());

  // Knot 2.
  times.push_back(1.0/3.0 * averageVelocity_);
  Position knot2(start_.x(), start_.y(), height);
  values.push_back(knot2.vector());

  // Knot 3.
  times.push_back(2.0/3.0 * averageVelocity_);
  Position knot3(target_.x(), target_.y(), height);
  values.push_back(knot3.vector());

  // Knot 4.
  times.push_back(averageVelocity_);
  values.push_back(target_.vector());
}

} /* namespace */

