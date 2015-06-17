/*
 * SwingSplineTrajectory.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/SwingSplineTrajectory.hpp>

namespace free_gait {

SwingSplineTrajectory::SwingSplineTrajectory() :
    trajectoryUpdated_(false),
    SwingTrajectoryBase()
{
}

SwingSplineTrajectory::~SwingSplineTrajectory()
{
}

std::unique_ptr<SwingTrajectoryBase> SwingSplineTrajectory::clone() const
{
  std::unique_ptr<SwingTrajectoryBase> pointer(new SwingSplineTrajectory(*this));
  return pointer;
}

bool SwingSplineTrajectory::updateStartPosition(const Position& startPosition)
{
  if (times_[0] == 0.0) {
    values_[0] = startPosition.vector();
  } else {
    times_.insert(times_.begin(), 0.0);
    values_.insert(values_.begin(), startPosition.vector());
  }
  trajectoryUpdated_ = false;
  return computeTrajectory();
}

const Position SwingSplineTrajectory::evaluate(const double phase)
{
  if (!trajectoryUpdated_) computeTrajectory();
  const double time = phase * getDuration();
  return Position(trajectory_.evaluate(time));
}

double SwingSplineTrajectory::getDuration() const
{
  //  return trajectory_.getMaxTime() - trajectory_.getMinTime();
  // We assume the swing spline trajectory always starts at time 0.0.
  return times_.back();
}

const Position SwingSplineTrajectory::getTarget() const
{
  return Position(trajectory_.evaluate(trajectory_.getMaxTime()));
}


bool SwingSplineTrajectory::computeTrajectory()
{
  trajectory_.fitCurve(times_, values_);
  trajectoryUpdated_ = true;
  return true;
}

std::ostream& operator<<(std::ostream& out, const SwingSplineTrajectory& swingSplineTrajectory)
{
  // TODO.
  return out;
}

} /* namespace */
