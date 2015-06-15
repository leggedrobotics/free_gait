/*
 * SwingProfile.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/SwingProfile.hpp"

// Roco
#include <roco/log/log_messages.hpp>

namespace free_gait {

SwingProfile::SwingProfile()
    : SwingTrajectoryBase(),
      height_(0.0),
      duration_(0.0),
      trajectoryUpdated_(false)
{
}

SwingProfile::~SwingProfile()
{
}

std::unique_ptr<SwingTrajectoryBase> SwingProfile::clone() const
{
  std::unique_ptr<SwingTrajectoryBase> pointer(new SwingProfile(*this));
  return pointer;
}

bool SwingProfile::updateStartPosition(const Position& startPosition)
{
  start_ = startPosition;
  trajectoryUpdated_ = false;
  return computeTrajectory();
}

const Position SwingProfile::evaluate(const double phase)
{
  if (!trajectoryUpdated_) computeTrajectory();
  const double time = phase * getDuration();
  return Position(trajectory_.evaluate(time));
}

double SwingProfile::getDuration() const
{
  return duration_;
}

void SwingProfile::setDuration(double duration)
{
  duration_ = duration;
  trajectoryUpdated_ = false;
}

double SwingProfile::getHeight() const
{
  return height_;
}

void SwingProfile::setHeight(double height)
{
  height_ = height;
  trajectoryUpdated_ = false;
}

const Position SwingProfile::getTarget() const
{
  return target_;
}

void SwingProfile::setTarget(const Position& target)
{
  target_ = target;
  trajectoryUpdated_ = false;
}

const std::string& SwingProfile::getType() const
{
  return type_;
}

void SwingProfile::setType(const std::string& type)
{
  type_ = type;
  trajectoryUpdated_ = false;
}

std::ostream& operator<<(std::ostream& out, const SwingProfile& swingProfile)
{
  out << "Target: " << swingProfile.target_ << std::endl;
  out << "Height: " << swingProfile.height_ << std::endl;
  out << "Duration: " << swingProfile.duration_ << std::endl;
  out << "Type: " << swingProfile.type_;
  return out;
}

bool SwingProfile::computeTrajectory()
{
  std::vector<Time> times;
  std::vector<ValueType> values;

  if (type_ == "triangle") {
    generateTriangleKnots(times, values);
  } else if (type_ == "square") {
    generateSquareKnots(times, values);
  } else {
    ROCO_ERROR_STREAM("Swing profile of type '" << type_ << "' not supported.");
    return false;
  }

  trajectory_.fitCurve(times, values);
  trajectoryUpdated_ = true;
  return true;
}

void SwingProfile::generateTriangleKnots(std::vector<Time>& times,
                                         std::vector<ValueType>& values) const
{
  // Knot 1.
  times.push_back(0.0);
  values.push_back(start_.vector());

  // Knot 2.
  times.push_back(0.5 * duration_);
  // Interpolate on the xy-plane.
  Position knot2 = start_ + 0.5 * (target_ - start_);
  // Apex height.
  double basis = start_.z() > target_.z() ? start_.z() : target_.z();
  knot2.z() = basis + height_;
  values.push_back(knot2.vector());

  // Knot 3.
  times.push_back(duration_);
  values.push_back(target_.vector());
}

void SwingProfile::generateSquareKnots(std::vector<Time>& times,
                                       std::vector<ValueType>& values) const
{
  double basis = start_.z() > target_.z() ? start_.z() : target_.z();
  double height = basis + height_;

  // Knot 1.
  times.push_back(0.0);
  values.push_back(start_.vector());

  // Knot 2.
  times.push_back(1.0/3.0 * duration_);
  Position knot2(start_.x(), start_.y(), height);
  values.push_back(knot2.vector());

  // Knot 3.
  times.push_back(2.0/3.0 * duration_);
  Position knot3(target_.x(), start_.y(), height);
  values.push_back(knot3.vector());

  // Knot 4.
  times.push_back(duration_);
  values.push_back(target_.vector());
}

} /* namespace */

