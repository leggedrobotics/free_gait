/*
 * SwingSplineTrajectory.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/SwingSplineTrajectory.hpp>

namespace free_gait {

SwingSplineTrajectory::SwingSplineTrajectory() : SwingTrajectoryBase()
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
  return true;
}

const Position SwingSplineTrajectory::evaluate(const double phase)
{
  const double time = phase * getDuration();
  return Position(trajectory_.evaluate(time));
}

double SwingSplineTrajectory::getDuration() const
{
  return trajectory_.getMaxTime() - trajectory_.getMinTime();
}

const Position SwingSplineTrajectory::getTarget() const
{
  return Position(trajectory_.evaluate(trajectory_.getMaxTime()));
}

std::ostream& operator<<(std::ostream& out, const SwingSplineTrajectory& swingSplineTrajectory)
{
  // TODO.
  return out;
}

} /* namespace */
