/*
 * SwingFootTrajectory.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/SwingFootTrajectory.hpp>

namespace free_gait {

SwingFootTrajectory::SwingFootTrajectory()
    : SwingTrajectoryBase(SwingTrajectoryType::FootTrajectory),
      trajectoryUpdated_(false)
{
}

SwingFootTrajectory::~SwingFootTrajectory()
{
}

std::unique_ptr<SwingTrajectoryBase> SwingFootTrajectory::clone() const
{
  std::unique_ptr<SwingTrajectoryBase> pointer(new SwingFootTrajectory(*this));
  return pointer;
}

bool SwingFootTrajectory::updateStartPosition(const Position& startPosition)
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

const Position SwingFootTrajectory::evaluate(const double phase)
{
  if (!trajectoryUpdated_) computeTrajectory();
  const double time = phase * getDuration();
  return Position(trajectory_.evaluate(time));
}

double SwingFootTrajectory::getDuration() const
{
  //  return trajectory_.getMaxTime() - trajectory_.getMinTime();
  // We assume the swing spline trajectory always starts at time 0.0.
  return times_.back();
}

const Position SwingFootTrajectory::getTarget() const
{
  return Position(trajectory_.evaluate(trajectory_.getMaxTime()));
}


bool SwingFootTrajectory::computeTrajectory()
{
  trajectory_.fitCurve(times_, values_);
  trajectoryUpdated_ = true;
  return true;
}

std::ostream& operator<<(std::ostream& out, const SwingFootTrajectory& swingFootTrajectory)
{
  // TODO.
  return out;
}

} /* namespace */
